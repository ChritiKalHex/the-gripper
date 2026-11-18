// gamepad_motors.c - SDL3 gamepad -> serial commands for the two-motor Arduino sketch (Linux)
// Usage: ./gamepad_motors /dev/ttyACM0
#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ---- Input mapping (edit these to remap) ----
#define TRIGGER_ON      8000   // 
#define TRIGGER_OFF     4000   //
#define CHUNK_STEPS     3000   // 
static const SDL_GamepadAxis   TRIGGER_AXIS[2]   = { SDL_GAMEPAD_AXIS_LEFT_TRIGGER,
                                                      SDL_GAMEPAD_AXIS_RIGHT_TRIGGER }; // left/right triggers to move motors 1/2
static const SDL_GamepadButton REVERSE_BUTTON[2] = { SDL_GAMEPAD_BUTTON_LEFT_SHOULDER,
                                                      SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER }; // hold to reverse direction of motor 1/2
// static const SDL_GamepadAxis CLAW_AXIS[] = { SDL_GAMEPAD_BUTTON_DPAD_DOWN,
//                                              SDL_GAMEPAD_BUTTON_DPAD_UP }; // hold to move claw
// static const....something something wrist movement with left stick.
// static const....something something base movement with right stick.

// ---- Serial port ----
static int port_open(const char *name) {
    int fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios t;
    tcgetattr(fd, &t);
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &t);
    return fd;
}

static int port_read(int fd, char *buf, int n) {
    int r = (int)read(fd, buf, n);
    return r > 0 ? r : 0;
}

static void port_write(int fd, const char *s) {
    (void)!write(fd, s, strlen(s));
}

// ---- App state (memory)----
typedef struct {
    int port;
    SDL_Gamepad *pad;
    Uint64 ready_at;      // Arduino resets when the port opens; wait before sending
    bool moving[2];
    bool wanted[2];
    char line[64];
    int  line_len;
} App;

static void send_cmd(App *a, const char *s) {
    port_write(a->port, s);
    SDL_Log("-> %s", s);
}

static void stop_all(App *a) {
    send_cmd(a, "x\n");
    a->moving[0] = a->moving[1] = false;
}

static void handle_reply(App *a, const char *line) { //PC reads what the Arduino sends back, and updates moving[] accordingly
    SDL_Log("<- %s", line);
    int n;
    if (sscanf(line, "Motor %d done", &n) == 1 && n >= 1 && n <= 2) {
        a->moving[n - 1] = false;
    } else if (strstr(line, "OVERHEAT")) {
        a->moving[0] = a->moving[1] = false;
    }
}

static void pump_serial(App *a) {
    char buf[64];
    int n;
    while ((n = port_read(a->port, buf, sizeof buf)) > 0) {
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                if (a->line_len > 0) {
                    a->line[a->line_len] = 0;
                    a->line_len = 0;
                    handle_reply(a, a->line);
                }
            } else if (a->line_len < (int)sizeof a->line - 1) {
                a->line[a->line_len++] = c;
            }
        }
    }
}

static void open_first_pad(App *a) {
    if (a->pad) return;
    int count = 0;
    SDL_JoystickID *ids = SDL_GetGamepads(&count);
    if (ids && count > 0) {
        a->pad = SDL_OpenGamepad(ids[0]);
        if (a->pad) SDL_Log("Gamepad: %s", SDL_GetGamepadName(a->pad));
    }
    SDL_free(ids);
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
    if (argc < 2) {
        SDL_Log("Usage: %s <serial port>  (e.g. /dev/ttyACM0)", argv[0]);
        return SDL_APP_FAILURE;
    }

    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    if (!SDL_Init(SDL_INIT_GAMEPAD)) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    App *a = (App *)SDL_calloc(1, sizeof(App));
    if (!a) return SDL_APP_FAILURE;

    a->port = port_open(argv[1]);
    if (a->port < 0) {
        SDL_Log("Couldn't open serial port %s (is it in use, or are you missing the dialout group?)", argv[1]);
        return SDL_APP_FAILURE;
    }
    a->ready_at = SDL_GetTicks() + 2000;

    open_first_pad(a);
    *appstate = a;
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
    App *a = (App *)appstate;
    switch (event->type) {
        case SDL_EVENT_QUIT:
            return SDL_APP_SUCCESS;
        case SDL_EVENT_GAMEPAD_ADDED:
            open_first_pad(a);
            break;
        case SDL_EVENT_GAMEPAD_REMOVED:
            if (a->pad && SDL_GetGamepadID(a->pad) == event->gdevice.which) {
                SDL_CloseGamepad(a->pad);
                a->pad = NULL;
                stop_all(a);              // fail safe: controller gone = motors stop
                SDL_Log("Gamepad disconnected");
            }
            break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
    App *a = (App *)appstate;
    pump_serial(a);

    if (SDL_GetTicks() >= a->ready_at && a->pad) {
        // 1) read triggers (with hysteresis)
        for (int i = 0; i < 2; i++) {
            Sint16 v = SDL_GetGamepadAxis(a->pad, TRIGGER_AXIS[i]);
            if (v > TRIGGER_ON)       a->wanted[i] = true;
            else if (v < TRIGGER_OFF) a->wanted[i] = false;
        }

        // 2) a released trigger -> stop (firmware's 'x' stops both axes)
        bool need_stop = false;
        for (int i = 0; i < 2; i++)
            if (!a->wanted[i] && a->moving[i]) need_stop = true;
        if (need_stop) stop_all(a);

        // 3) start (or restart) any held trigger that isn't running
        for (int i = 0; i < 2; i++) {
            if (a->wanted[i] && !a->moving[i]) {
                bool rev = SDL_GetGamepadButton(a->pad, REVERSE_BUTTON[i]);
                char cmd[32];
                SDL_snprintf(cmd, sizeof cmd, "m%d %d\n", i + 1, rev ? -CHUNK_STEPS : CHUNK_STEPS);
                send_cmd(a, cmd);
                a->moving[i] = true;
            }
        }
    }

    SDL_Delay(5);
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
    App *a = (App *)appstate;
    if (a) {
        if (a->port >= 0) { stop_all(a); close(a->port); }
        if (a->pad) SDL_CloseGamepad(a->pad);
        SDL_free(a);
    }
}
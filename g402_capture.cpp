#include <SDL2/SDL.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cerrno>

#define ARRAY_SIZE 26
#define PIXEL_COUNT (ARRAY_SIZE * ARRAY_SIZE)
#define FRAME_SIZE (1 + PIXEL_COUNT)
#define FRAME_HEADER 0x01
#define PIXEL_SCALE 5

int main() {
    // Open CDC serial device
    int fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    // Configure serial port as raw
    termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);

    // Init SDL
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Sensor Frame",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        ARRAY_SIZE * PIXEL_SCALE, ARRAY_SIZE * PIXEL_SCALE, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Buffers
    uint8_t buf[FRAME_SIZE];
    uint8_t image[PIXEL_COUNT];
    uint32_t frame_count = 0;
    uint32_t fps_count = 0;
    uint32_t last_time = SDL_GetTicks();

    SDL_Event e;
    bool quit = false;

    while (!quit) {
        // Sync to header
        ssize_t n = read(fd, buf, 1);
        if (n <= 0 || buf[0] != FRAME_HEADER) continue;

        // Read the rest of the frame
        ssize_t received = 0;
        while (received < PIXEL_COUNT) {
            ssize_t r = read(fd, image + received, PIXEL_COUNT - received);
            if (r > 0) received += r;
        }

        // FPS counting
        frame_count++;
        fps_count++;
        uint32_t now = SDL_GetTicks();
        if (now - last_time >= 1000) {
            float fps = fps_count * 1000.0f / (now - last_time);
            printf("Frame #%u | FPS: %.2f\n", frame_count, fps);  // 통일된 형식
            last_time = now;
            fps_count = 0;
        }

        // Render frame
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        for (int y = 0; y < ARRAY_SIZE; y++) {
            for (int x = 0; x < ARRAY_SIZE; x++) {
                uint8_t v = image[y * ARRAY_SIZE + x];
                SDL_SetRenderDrawColor(renderer, v, v, v, 255);
                SDL_Rect r = { x * PIXEL_SCALE, y * PIXEL_SCALE, PIXEL_SCALE, PIXEL_SCALE };
                SDL_RenderFillRect(renderer, &r);
            }
        }
        SDL_RenderPresent(renderer);

        // Handle quit event
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
        }
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    close(fd);
    return 0;
}

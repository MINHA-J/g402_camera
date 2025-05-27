/* SystemClock_Config() and Error_Handler() remain unchanged */

// PC-side C++ Receiver for full-frame CDC protocol
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <sys/types.h>

#define ARRAY_SIZE 26
#define PIXEL_SIZE 5
#define FRAME_SIZE (1 + ARRAY_SIZE * ARRAY_SIZE)
#define FRAME_HEADER 0x01

int main() {
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    // Open CDC device
    int fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);
    if (fd < 0) { perror("cannot open /dev/ttyACM0"); return 1; }
    // Ensure blocking mode
    int flags = fcntl(fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    fcntl(fd, F_SETFL, flags);
    // Configure raw mode
    struct termios tio = {};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    // VMIN must fit in cc_t (unsigned char), so use 1 and handle full size manually
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);

    // SDL initialization
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init error: %s", SDL_GetError());
            close(fd);
        return 1;
    }
    window = SDL_CreateWindow("G402 Capture", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        ARRAY_SIZE * PIXEL_SIZE, ARRAY_SIZE * PIXEL_SIZE, SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "SDL_CreateRenderer error: %s", SDL_GetError());
            SDL_DestroyWindow(window);
        SDL_Quit();
        close(fd);
        return 1;
    }

    uint8_t frame[FRAME_SIZE];
    uint8_t img[ARRAY_SIZE * ARRAY_SIZE];
    uint32_t frame_count = 0, fps_count = 0;
    uint32_t last_time = SDL_GetTicks();
    SDL_Event e;
    bool quit = false;

    while (!quit) {
        // Read one full frame
        ssize_t total = 0;
        while (total < FRAME_SIZE) {
            ssize_t n = read(fd, frame + total, FRAME_SIZE - total);
            if (n < 0) {
                if (errno == EAGAIN) continue;
                perror("read error");
                quit = true;
                break;
            }
            total += n;
        }
        if (quit) break;
        // Sync on header
        if (frame[0] != FRAME_HEADER) continue;
        memcpy(img, &frame[1], ARRAY_SIZE * ARRAY_SIZE);

        // FPS counting
        frame_count++;
        fps_count++;
        uint32_t now = SDL_GetTicks();
        if (now - last_time >= 1000) {
            float fps = fps_count * 1000.0f / (now - last_time);
            printf("Frame #%u | FPS: %.2f\n", frame_count, fps);
                last_time = now;
            fps_count = 0;
        }

        // Render image
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        for (int y = 0; y < ARRAY_SIZE; y++) {
            for (int x = 0; x < ARRAY_SIZE; x++) {
                uint8_t v = img[y * ARRAY_SIZE + x];
                SDL_SetRenderDrawColor(renderer, v, v, v, 255);
                SDL_Rect r = { x * PIXEL_SIZE, y * PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE };
                SDL_RenderFillRect(renderer, &r);
            }
        }
        SDL_RenderPresent(renderer);
        while (SDL_PollEvent(&e)) if (e.type == SDL_QUIT) quit = true;
    }

    close(fd);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

#define ARRAY_SIZE 26
#define PIXEL_SIZE 5
#define FRAME_LINE_SIZE (2 + ARRAY_SIZE * 2)  // start + line + 52 = 54

struct __attribute__((__packed__)) FRAME {
    uint8_t start;
    uint8_t line;
    uint8_t linebuff[ARRAY_SIZE * 2];
};

int main() {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;

    // CDC 장치 열기
    int fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("cannot open /dev/ttyACM0");
        return 1;
    }

    // SDL 초기화
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        close(fd);
        return 1;
    }

    window = SDL_CreateWindow(
        "G402 Capture",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        ARRAY_SIZE * PIXEL_SIZE, ARRAY_SIZE * PIXEL_SIZE,
        SDL_WINDOW_SHOWN
    );

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "SDL_CreateRenderer error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        close(fd);
        return 1;
    }

    bool quit = false;
    SDL_Event e;

    uint8_t arr[ARRAY_SIZE * ARRAY_SIZE] = { 0 };  // 최종 이미지 버퍼

    uint32_t frame_count = 0;
    uint32_t fps_frame_count = 0;
    uint32_t last_fps_time = SDL_GetTicks();

    while (!quit) {
        uint8_t temp_buffer[FRAME_LINE_SIZE];
        FRAME* frame = (FRAME*)temp_buffer;

        // 프레임 시작 라인 찾기 (start == 1)
        do {
            ssize_t received = 0;
            while (received < FRAME_LINE_SIZE) {
                ssize_t n = read(fd, temp_buffer + received, FRAME_LINE_SIZE - received);
                if (n > 0) received += n;
                else {
                    perror("read error (start)");
                    quit = true;
                    break;
                }
            }
        } while (!frame->start && !quit);

        if (quit) break;

        // 프레임 버퍼 초기화
        memset(arr, 0, sizeof(arr));
        memcpy(&arr[frame->line * ARRAY_SIZE * 2], frame->linebuff, ARRAY_SIZE * 2);

        // 나머지 12줄 수신
        for (int i = 1; i < ARRAY_SIZE / 2; i++) {
            ssize_t received = 0;
            while (received < FRAME_LINE_SIZE) {
                ssize_t n = read(fd, temp_buffer + received, FRAME_LINE_SIZE - received);
                if (n > 0) received += n;
                else {
                    perror("read error (next line)");
                    quit = true;
                    break;
                }
            }
            if (quit) break;
            for (int j = 0; j < ARRAY_SIZE; j++) {
                arr[frame->line * ARRAY_SIZE + j] = frame->linebuff[j * 2]; // 픽셀만 추출
            }
        }

        // FPS 측정
        frame_count++;
        fps_frame_count++;
        uint32_t now = SDL_GetTicks();
        if (now - last_fps_time >= 1000) {
            float fps = fps_frame_count * 1000.0f / (now - last_fps_time);
            printf("Frame #%u | FPS: %.2f\n", frame_count, fps);
            last_fps_time = now;
            fps_frame_count = 0;
        }

        // 화면 렌더링
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        for (uint32_t x = 0; x < ARRAY_SIZE; x++) {
            for (uint32_t y = 0; y < ARRAY_SIZE; y++) {
                uint8_t pixel = arr[ARRAY_SIZE * x + y];
                SDL_SetRenderDrawColor(renderer, pixel * 2, pixel * 2, pixel * 2, 255);
                SDL_Rect r = {
                    .x = y * PIXEL_SIZE,
                    .y = x * PIXEL_SIZE,
                    .w = PIXEL_SIZE,
                    .h = PIXEL_SIZE
                };
                SDL_RenderFillRect(renderer, &r);
            }
        }

        SDL_RenderPresent(renderer);

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT)
                quit = true;
        }
    }

    close(fd);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

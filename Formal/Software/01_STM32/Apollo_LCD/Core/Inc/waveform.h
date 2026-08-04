#ifndef WAVEFORM_H
#define WAVEFORM_H

#include <stdint.h>

#define WAVEFORM_SAMPLE_COUNT 181U

void Waveform_Init(uint8_t touchAvailable);
void Waveform_SubmitFrame(const uint16_t samples[WAVEFORM_SAMPLE_COUNT],
                          uint32_t frequencyHz, uint32_t sampleDiv,
                          uint32_t periodCycles,
                          uint8_t flags, uint8_t sequence);
void Waveform_Update(void);
void Waveform_SetTouchAvailable(uint8_t touchAvailable);
void Waveform_SetLinkStats(uint32_t bytesReceived, uint32_t validFrames,
                           uint32_t totalErrors);
void Waveform_HandleTouch(uint16_t x, uint16_t y);
void Waveform_ShowTouch(uint16_t x, uint16_t y);

#endif

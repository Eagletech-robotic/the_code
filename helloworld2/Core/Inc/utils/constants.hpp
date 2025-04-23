#pragma once

static constexpr int SQUARE_SIZE_CM = 4;
static constexpr float SQUARE_SIZE_M = SQUARE_SIZE_CM / 100.0f;

static constexpr int FIELD_WIDTH_SQ = 300 / SQUARE_SIZE_CM;
static constexpr int FIELD_HEIGHT_SQ = 200 / SQUARE_SIZE_CM;
static constexpr float FIELD_WIDTH_M = FIELD_WIDTH_SQ * SQUARE_SIZE_M;
static constexpr float FIELD_HEIGHT_M = FIELD_HEIGHT_SQ * SQUARE_SIZE_M;

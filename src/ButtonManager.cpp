#include "ButtonManager.h"
#include "BleHidController.h"
#include <Arduino.h>

ButtonManager::ButtonManager() : m_hidController(nullptr) {
    // Initialize button configurations
    //@formatter:off
    buttons[0]  = {BTN_A_PIN,               ButtonType::BUTTON, ClassicBtn::A,               FxKey::A,               VpxKey::A,               HIGH, 0};
    buttons[1]  = {BTN_B_PIN,               ButtonType::BUTTON, ClassicBtn::B,               FxKey::B,               VpxKey::B,               HIGH, 0};
    buttons[2]  = {BTN_X_PIN,               ButtonType::BUTTON, ClassicBtn::X,               FxKey::X,               VpxKey::X,               HIGH, 0};
    buttons[3]  = {BTN_Y_PIN,               ButtonType::BUTTON, ClassicBtn::Y,               FxKey::Y,               VpxKey::Y,               HIGH, 0};
    buttons[4]  = {BTN_SELECT_PIN,          ButtonType::BUTTON, ClassicBtn::SELECT,          FxKey::SELECT,          VpxKey::SELECT,          HIGH, 0};
    buttons[5]  = {BTN_START_PIN,           ButtonType::BUTTON, ClassicBtn::START,           FxKey::START,           VpxKey::START,           HIGH, 0};
    buttons[6]  = {BTN_LAUNCH_PIN,          ButtonType::BUTTON, ClassicBtn::LAUNCH,          FxKey::LAUNCH,          VpxKey::LAUNCH,          HIGH, 0};
    buttons[7]  = {BTN_LEFT_FLIPPER_PIN,    ButtonType::BUTTON, ClassicBtn::LEFT_FLIPPER,    FxKey::LEFT_FLIPPER,    VpxKey::LEFT_FLIPPER,    HIGH, 0};
    buttons[8]  = {BTN_RIGHT_FLIPPER_PIN,   ButtonType::BUTTON, ClassicBtn::RIGHT_FLIPPER,   FxKey::RIGHT_FLIPPER,   VpxKey::RIGHT_FLIPPER,   HIGH, 0};
    buttons[9]  = {BTN_LEFT_MAGNASAVE_PIN,  ButtonType::BUTTON, ClassicBtn::LEFT_MAGNASAVE,  FxKey::LEFT_MAGNASAVE,  VpxKey::LEFT_MAGNASAVE,  HIGH, 0};
    buttons[10] = {BTN_RIGHT_MAGNASAVE_PIN, ButtonType::BUTTON, ClassicBtn::RIGHT_MAGNASAVE, FxKey::RIGHT_MAGNASAVE, VpxKey::RIGHT_MAGNASAVE, HIGH, 0};
    buttons[11] = {DPAD_UP_PIN,             ButtonType::DPAD,   ClassicBtn::UP,              FxKey::UP,              VpxKey::UP,              HIGH, 0};
    buttons[12] = {DPAD_DOWN_PIN,           ButtonType::DPAD,   ClassicBtn::DOWN,            FxKey::DOWN,            VpxKey::DOWN,            HIGH, 0};
    buttons[13] = {DPAD_LEFT_PIN,           ButtonType::DPAD,   ClassicBtn::LEFT,            FxKey::LEFT,            VpxKey::LEFT,            HIGH, 0};
    buttons[14] = {DPAD_RIGHT_PIN,          ButtonType::DPAD,   ClassicBtn::RIGHT,           FxKey::RIGHT,           VpxKey::RIGHT,           HIGH, 0};
    //@formatter:on
}

void ButtonManager::begin(BleHidController& hidController) {
    m_hidController = &hidController;
    for (const auto& button : buttons) {
        pinMode(button.pin, INPUT_PULLUP);
    }
}

void ButtonManager::handle(ControllerMode mode) {
    for (auto& button : buttons) {
        handleButton(button, mode);
    }
}

void ButtonManager::handleButton(ButtonInfo& button, ControllerMode mode) {
    const auto currentMillis = millis();
    if (currentMillis - button.lastDebounceTime < BTN_DEBOUNCE_MS) {
        return;
    }

    if (const int reading = digitalRead(button.pin); reading != button.state) {
        button.state            = reading;
        button.lastDebounceTime = currentMillis;
        performButtonAction(getButtonAction(button, mode), button.state == LOW);
    }
}

void ButtonManager::performButtonAction(const ButtonAction& action, const bool isPressed) {
    if (!m_hidController) return;
    
    switch (action.type) {
        //@formatter:off
        case ActionType::KEYBOARD_KEY:
            if (isPressed)                  m_hidController->keyPress(action.keyCode);
            else                            m_hidController->keyRelease(action.keyCode);
            break;
        case ActionType::GAMEPAD_BUTTON:
            if (isPressed)                  m_hidController->buttonPress(action.buttonCode);
            else                            m_hidController->buttonRelease(action.buttonCode);
            break;
        case ActionType::GAMEPAD_DPAD:
            if (isPressed)                  m_hidController->dpadPress(action.dpadValue);
            else                            m_hidController->dpadRelease();
            break;
        case ActionType::GAMEPAD_LT:
            if (isPressed)                  m_hidController->setLeftTrigger(32767, false);
            else                            m_hidController->setLeftTrigger(0, false);
            break;
        case ActionType::GAMEPAD_RT:
            if (isPressed)                  m_hidController->setRightTrigger(32767, false);
            else                            m_hidController->setRightTrigger(0, false);
            break;
        case ActionType::NONE:
        default:
            break;
        //@formatter:on
    }
}

ButtonAction ButtonManager::getButtonAction(const ButtonInfo& button, ControllerMode mode) {
    switch (mode) {
        //@formatter:off
        case ControllerMode::FX:
            return {.type = ActionType::KEYBOARD_KEY, .keyCode = static_cast<uint8_t>(button.fxKey)};
        case ControllerMode::VPX:
            return {.type = ActionType::KEYBOARD_KEY, .keyCode = static_cast<uint8_t>(button.vpxKey)};
        case ControllerMode::CLASSIC:
            if (button.type == ButtonType::DPAD)
                return {.type = ActionType::GAMEPAD_DPAD, .dpadValue = static_cast<uint8_t>(button.classicCode)};
            if (static_cast<uint16_t>(button.classicCode) == TRIGGER_LEFT)
                return {.type = ActionType::GAMEPAD_LT};
            if (static_cast<uint16_t>(button.classicCode) == TRIGGER_RIGHT)
                return {.type = ActionType::GAMEPAD_RT};
            return {.type = ActionType::GAMEPAD_BUTTON, .buttonCode = static_cast<uint16_t>(button.classicCode)};
        default:
            return {.type = ActionType::NONE};
        //@formatter:on
    }
}

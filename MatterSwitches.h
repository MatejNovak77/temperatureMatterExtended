#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <Matter.h>
#include <MatterLightbulb.h>

// -----------------------------
// EEPROM helpers
// -----------------------------
static inline uint8_t eepromReadByteOrDefault(int addr, uint8_t fallback) {
  uint8_t b = EEPROM.read(addr);
  return (b == 0xFF) ? fallback : b;
}
static inline void eepromWriteByteChanged(int addr, uint8_t value) {
  if (EEPROM.read(addr) != value) EEPROM.write(addr, value);
}

// -----------------------------
// VirtualSwitch (polling + EEPROM)
// -----------------------------
class VirtualSwitch {
public:
  VirtualSwitch(const char* displayName, int eepromStateAddr, bool defaultOn = false)
  : _name(displayName), _stateAddr(eepromStateAddr), _defaultOn(defaultOn) {}

  void begin() {
    _bulb.begin();
    const bool initial = eepromReadByteOrDefault(_stateAddr, _defaultOn ? 1 : 0) != 0;
    _bulb.set_onoff(initial);
    _lastState = _bulb.get_onoff();
  }

  // Call in loop()
  void update() {
    bool cur = _bulb.get_onoff();
    if (cur != _lastState) {
      _lastState = cur;
      eepromWriteByteChanged(_stateAddr, cur ? 1 : 0);
      if (_onChange) _onChange(cur);
    }
  }

  // Programmatic control
  void set(bool on) {
    if (_bulb.get_onoff() != on) {
      _bulb.set_onoff(on);
      // Persistence is handled uniformly in update()
    }
  }
  bool get() { return _bulb.get_onoff(); }
  void toggle() { set(!get()); }

  void onChange(void (*cb)(bool)) { _onChange = cb; }
  const char* name() const { return _name; }
  MatterLightbulb& raw() { return _bulb; }

private:
  const char* _name;
  int _stateAddr;
  bool _defaultOn = false;

  MatterLightbulb _bulb;
  bool _lastState = false;
  void (*_onChange)(bool) = nullptr;
};

// -----------------------------
// ModeSwitch (maps ON -> one of 1..N modes)
// -----------------------------
class ModeSwitch {
public:
  ModeSwitch(const char* displayName,
             int eepromStateAddr,
             int eepromIdxAddr,
             const uint8_t* modes, size_t modesCount,
             bool defaultOn = false,
             uint8_t defaultMode = 0)
  : _vs(displayName, eepromStateAddr, defaultOn),
    _idxAddr(eepromIdxAddr),
    _modes(modes), _modesCount(modesCount) {}

  void begin() {
    _vs.begin();
    uint8_t idx = eepromReadByteOrDefault(_idxAddr, 0);
    if (idx >= _modesCount) idx = 0;
    _idx = idx;
    _vs.onChange(+[](bool){}); // no-op; we poll via update()
  }

  void update() { _vs.update(); }

  // 0 if OFF, else active mode ID
  uint8_t currentMode() { return _vs.get() ? _modes[_idx] : 0; }

  void forceMode(uint8_t mode) {
    int pos = findMode(mode);
    if (pos >= 0) {
      _idx = (uint8_t)pos;
      eepromWriteByteChanged(_idxAddr, _idx);
      if (!_vs.get()) _vs.set(true);
    }
  }

  void setOn(bool on) { _vs.set(on); }
  bool isOn() { return _vs.get(); }

  void nextSubMode() {
    if (_modesCount == 0) return;
    _idx = (uint8_t)((_idx + 1) % _modesCount);
    eepromWriteByteChanged(_idxAddr, _idx);
    if (!_vs.get()) _vs.set(true);
  }

  const uint8_t* modesList() const { return _modes; }
  size_t modesCount() const { return _modesCount; }

private:
  int findMode(uint8_t mode) const {
    for (size_t i = 0; i < _modesCount; ++i) if (_modes[i] == mode) return (int)i;
    return -1;
  }

  VirtualSwitch _vs;
  int _idxAddr;
  uint8_t _idx = 0;

  const uint8_t* _modes;
  size_t _modesCount;
};

// -----------------------------
// MultiModeSelector (polling)
// -----------------------------
class MultiModeSelector {
public:
  explicit MultiModeSelector(uint8_t totalModes) : _totalModes(totalModes) {}

  void attach(ModeSwitch* m) { _members[_count++] = m; }
  void onGroupChange(void (*cb)(uint8_t)) { _onGroupChange = cb; }
  uint8_t currentMode() const { return _groupMode; }

  void updateGroup() {
    // 1) update members
    for (size_t i = 0; i < _count; ++i) _members[i]->update();

    // 2) find ON members
    int onIdx = -1;
    bool conflict = false;
    for (size_t i = 0; i < _count; ++i) {
      if (_members[i]->isOn()) {
        if (onIdx == -1) onIdx = (int)i;
        else conflict = true;
      }
    }

    // 3) enforce exclusivity
    if (conflict) {
      for (size_t i = 0; i < _count; ++i) if ((int)i != onIdx) _members[i]->setOn(false);
    }

    // 4) compute aggregate mode
    uint8_t newMode = 0;
    if (onIdx >= 0) {
      newMode = _members[onIdx]->currentMode();
      if (newMode > _totalModes) newMode = 0;
    }
    if (newMode != _groupMode) {
      _groupMode = newMode;
      if (_onGroupChange) _onGroupChange(_groupMode);
    }
  }

  void forceMode(uint8_t mode) {
    if (mode == 0) {
      for (size_t i = 0; i < _count; ++i) _members[i]->setOn(false);
      return;
    }
    for (size_t i = 0; i < _count; ++i) {
      const uint8_t* list = _members[i]->modesList();
      size_t n = _members[i]->modesCount();
      for (size_t j = 0; j < n; ++j) {
        if (list[j] == mode) {
          _members[i]->forceMode(mode);
          for (size_t k = 0; k < _count; ++k) if (k != i) _members[k]->setOn(false);
          return;
        }
      }
    }
  }

private:
  ModeSwitch* _members[8] = {nullptr};
  size_t _count = 0;
  uint8_t _totalModes = 0;
  uint8_t _groupMode = 0;
  void (*_onGroupChange)(uint8_t) = nullptr;
};



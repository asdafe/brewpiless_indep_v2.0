# BrewPiLess — Independent Heat/Cool Modification (Source Package)

Bu pakette `feature/independent-heat-cool` branch'inde yaptığım değişikliklerin
kaynak dosyaları var. 

## İçerik

```
src/                      ← C++ firmware dosyaları (6 dosya)
  TempControl.h           ← enum, MODE_INDEPENDENT, modeIsIndependent()
  TempControl.cpp         ← updatePID / updateState / detectPeaks bağımsız mod
  BrewKeeper.cpp          ← profil sürücüsü: 'i' modunda da çalışır
  BrewPiProxy.cpp         ← HEAT_AND_COOL state'i için getStatusTime fix
  DisplayLcd.cpp          ← LCD'de "Heat+Cool" gösterimi
  Version.h               ← 0.3.0-indep

htmljs/src/               ← Web UI kaynak dosyaları
  control.tmpl.html       ← Yeni "Independent" sekmesi (navbar + body)
  control_s.tmpl.html     ← Aynısı, küçük ekran versiyonu
  js/script-control.js    ← modekeeper: 5. mod + apply handler
  locales/*.json          ← 7 dil dosyası (yeni anahtarlar eklendi)

wdoc/                     ← Önceden derlenmiş C header'lar (gz web UI binary)
  english_*.h             ← Bunları src/ ile aynı build path'ine koy
  chinese_*.h                → platformio otomatik wdoc/*.h'i include eder
  ... 49 dosya toplam

## Değişen dosyaların kısa özeti

### src/TempControl.h
- `MODE_INDEPENDENT 'i'` macro eklendi
- `enum states`'e `HEAT_AND_COOL = 10` eklendi
- `modeIsIndependent()` helper eklendi

### src/TempControl.cpp
- `updatePID()`: bağımsız modda no-op
- `updateState()`: bağımsız mod için iki paralel on/off döngüsü
- `detectPeaks()`: bağımsız modda atlanır
- `stateIsCooling()` / `stateIsHeating()`: HEAT_AND_COOL'u da kapsar

### src/BrewKeeper.cpp
- `keep()`: mode 'i' ise profili de çalıştır
- `setModeFromRemote()`: 'i' moduna geçildiğinde profil başlangıç zamanını set et

### src/BrewPiProxy.cpp
- `getStatusTime()`: HEAT_AND_COOL state'i için sinceIdleTime kullan

### src/DisplayLcd.cpp
- LCD'de HEAT_AND_COOL için "Heat+Cool" string'i
- printState sayaç bloğu: HEAT_AND_COOL'u da dahil et

### src/Version.h
- 0.2.4 → 0.3.0-indep

### htmljs/src/control.tmpl.html + control_s.tmpl.html
- 5. nav butonu: "Independent"
- Yeni `<div id="independent-s">`: iki input (cool-t, heat-t) + açıklama

### htmljs/src/js/script-control.js
- `modekeeper.modes` array'ine "independent" eklendi
- `apply()`: 'independent' için iki değer oku, `j{mode:i, beerSet:X, fridgeSet:Y}` gönder

### htmljs/src/locales/*.json
- `control_independent`, `control_setheattemp`, `control_setcooltemp`,
  `control_independent_help` anahtarları eklendi (7 dil)


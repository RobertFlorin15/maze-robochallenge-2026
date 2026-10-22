# MazeRunner – Robochallenge Maze

Firmware ESP32 pentru proba **Maze** (regulament: robochallenge.ro/regulation/maze).

## Ce spune regulamentul (și ce înseamnă pentru cod)

| Regulă | Consecință în firmware |
|---|---|
| Robot max **10 × 10 cm**, fără limită de greutate | geometria din `config.h` |
| Culoare și porți de **30 cm**, pereți de **15 cm**, albi/deschiși | praguri calculate din `CELL_MM = 300` |
| **2 roboți** pornesc simultan din 2 intrări simetrice; traseul devine comun; câștigă **primul care iese** | viteză maximă + detecție adversar + detecție ieșire |
| Fundături; dacă te blochezi, ești eliberat cu penalizare | deblocare automată (`stuckCheck`) |
| Ieși **doar** prin zona de ieșire marcată | alegerea corectă a mâinii (vezi mai jos) |
| Meci de **3 min**; fără ieșire câștigă cel mai aproape de ieșire | – |
| **Modul Start/Stop IR obligatoriu** (startmodule.com): START=0 la pornire, 1=start, 0=stop; după stop robotul rămâne blocat | implementat exact după Anexa 2 |
| Fără marcaje vizuale pe pistă; "beacons" permise | – |

## Strategia

**Urmărire de perete** cu mâna stângă sau dreaptă. Intrările și ieșirea sunt în peretele exterior, deci urmărind peretele exterior ajungi garantat la o deschidere în el. **Atenție:** urmărind peretele într-un sens poți ajunge întâi la intrarea adversarului. Cum startul e simetric, pentru un start una dintre mâini duce la ieșire, iar pentru celălalt start, cealaltă mână.

➡️ **Înainte de fiecare meci uită-te pe harta labirintului și pe ce intrare ești pus, apoi alege mâna** cu butonul BOOT (apăsare scurtă). LED-ul arată: **1 clipire = stânga**, **2 clipiri = dreapta**. Alegerea se salvează în flash.

Ce face robotul:
- **Regulator** pe distanța laterală + **corecție de unghi** din panta distanței laterale (cât se schimbă distanța până la perete pe mm parcurs); se centrează pe ambii pereți când există, pe unul singur când lipsește celălalt, iar fără pereți își ține direcția pe encodere (fără encodere merge în buclă deschisă și atunci contează `MOTOR_TRIM`).
- **Buclă de viteză PI** pe fiecare roată (cu encodere): comanda 0..1 devine viteză țintă, deci roțile merg la fel indiferent de baterie, frecări sau diferențele dintre motoare.
- **Frânare** proporțională cu distanța frontală, oprire exact în centrul celulei.
- **Gol pe partea mâinii** → avansează până în centrul golului (calculat din geometria senzorilor) → pivotează → intră în noul culoar.
- **Perete în față** → mâna liberă? → cealaltă parte liberă? → altfel întoarcere 180°.
- **Adversar**: dacă obstacolul din față se apropie mai repede decât mergem noi, sau apare brusc, robotul **așteaptă** până la 2,2 s în loc să se întoarcă (o întoarcere inutilă te costă mult timp).
- **Ieșire**: când toți cei 3 senzori (L, F, R) văd peste 90 cm pe cel puțin 10 cm de mers, robotul mai merge 18 cm ca să fie complet afară, se oprește și verifică din nou. Dacă vede iar pereți, a fost o alarmă falsă (ex. o intersecție cu culoare lungi) și continuă.
- Senzorii ToF sunt citiți pe **core 0**, controlul rulează la **200 Hz** pe core 1.

## Hardware recomandat și cablaj

- **Driver motoare: DRV8833** (ideal pentru N20, 2 pini PWM/motor). TB6612 merge și el, dar are nevoie de 7 pini.
- **GA12-N20 cu encoder magnetic** (6 fire, configurația actuală, `USE_ENCODERS 1`): distanțele și virajele se măsoară pe encodere, fiecare roată are buclă de viteză PI (`SPEED_LOOP`), iar fără pereți robotul își ține direcția tot pe encodere. Fără encodere pune `USE_ENCODERS 0` – atunci totul merge pe timp, corectat de ToF.
- **Encoderele se alimentează la 3,3 V**, nu la 5 V: ieșirile A/B urmează tensiunea de alimentare, iar ESP32 nu suportă 5 V pe pini. Fire tipice: roșu/negru = motor (M+/M−, la DRV8833), albastru = VCC encoder (3,3 V), verde = GND, galben/alb = A/B (culorile diferă între producători – verifică marcajele de pe plăcuța encoderului).
- **Divizor pentru baterie** (opțional cu encodere, recomandat fără): 100 kΩ de la +BAT la GPIO36, 47 kΩ de la GPIO36 la GND. Codul scalează PWM-ul după tensiune, ca robotul să se comporte la fel și cu bateria pe jumătate descărcată. Fără divizor pune `USE_VBAT 0`.
- Alimentare: LiPo 2S + regulator 3,3 V pentru ESP32 și senzori. Dacă motoarele sunt de 6 V, lasă `MOTOR_MAX_DUTY ≈ 0.8`.
- Modul Start/Stop: de la magazinul partener sau startmodule.com (VCC, GND, START, KILL).

| Semnal | GPIO |
|---|---|
| I2C SDA / SCL (toți ToF) | 21 / 22 (pull-up 4,7 kΩ dacă modulele nu au) |
| XSHUT ToF: Stânga, Față, Dreapta | 16, 18, 23 |
| DRV8833 motor stâng IN1 / IN2 | 25 / 26 |
| DRV8833 motor drept IN1 / IN2 | 27 / 14 |
| Encoder stâng A / B | 32 / 33 |
| Encoder drept A / B | 4 / 13 |
| Encoder VCC / GND (ambele) | 3,3 V / GND |
| Start module START / KILL | 34 / 35 |
| Tensiune baterie (divizor 100k/47k) | 36 |
| LED / buton | 2 / 0 (BOOT) |

**Montarea senzorilor (3× VL53L3CX):** L și R la exact 90° (perpendicular pe direcția de mers, contează pentru estimarea unghiului), F drept înainte, toți la ~5–8 cm înălțime (pereții au 15 cm), cât mai în față și cât mai simetric. Măsoară apoi offset-urile și pune-le în `config.h` (vezi mai jos „Referință → 2. Geometrie”).

## Instalare

1. Arduino IDE → Boards Manager: **esp32 by Espressif** (2.x sau 3.x). Placa: *ESP32 Dev Module*.
2. Library Manager: **STM32duino VL53L3CX** (se instalează cu tot ce-i trebuie).
3. Deschide `MazeRunner/MazeRunner.ino`, verifică pinii din `config.h`, încarcă.
4. Pe core-ul 2.x, dacă senzorii dau eroare la init, mărește `I2C_BUFFER_LENGTH` la 256 în `Wire.h` (pe 3.x codul o face singur).

## Unelte de calibrare

### Comenzi Serial

Deschide Serial Monitor la **115200**. Comenzile merg **doar înainte de start** (robotul așteaptă semnalul START):

| Comandă | Ce face |
|---|---|
| `s` | afișează cei 3 ToF, encoderele (impulsuri și mm), tensiunea bateriei și factorul de compensare |
| `l` / `r` | alege mâna stângă / dreaptă (se salvează în flash) |
| `9` | pivot 90° spre stânga |
| `8` | pivot 180° |
| `0` | pivot 360° (doar verificare) |
| `m` | mers drept o celulă (300 mm) la viteza 0,5, cu regulatorul de perete activ |
| `e` | test encodere – **cu roțile în aer** |

### Telemetrie

Cu `DEBUG_SERIAL 1`, în timpul cursei se afișează de 10 ori pe secundă:

```
FOLLOW   L 112 F 850 R 110 | odo   1234 | out 0.78 0.80 | v  470  482
```

- **starea**: `FOLLOW` (urmărește peretele), `ADVANCE` (avansează spre un gol), `PIVOT`, `ENTER` (intră în culoar după viraj), `DECIDE` (perete în față), `DEADEND` (fundătură, întoarcere 180°), `OPP_WAIT` (așteaptă adversarul), `UNSTUCK` (deblocare), `EXIT`, `DONE`;
- **L F R**: distanțele ToF în mm;
- **odo**: distanța parcursă (mm, media celor două roți);
- **out**: comanda pe fiecare roată (-1..1) după limitarea accelerației;
- **v**: viteza măsurată pe fiecare roată (mm/s). Viteza țintă e `out × MM_PER_S_AT_FULL`.

### LED și buton

- În așteptare: **1 clipire = mâna stângă**, **2 clipiri = mâna dreaptă**.
- Apăsare scurtă pe BOOT (sub 0,8 s) = schimbă mâna.
- Clipire rapidă continuă = eroare la pornire (START era 1, sau un senzor ToF nu a pornit; vezi Serial).
- Clipire lentă (0,5 s) = robotul a primit STOP; trebuie repornit de la alimentare.
- Clipire rapidă după ieșire = gata, așteaptă STOP.

## Calibrare (pe ordine)

Calibrează pe aceeași suprafață ca la concurs (lemn mat), cu roțile curate și bateria încărcată.

### Cu encodere (configurația actuală)

1. **Bateria** (`s`): compară `Vbat` cu un multimetru. Dacă diferă, corectează `VBAT_DIV` (`VBAT_DIV_nou = VBAT_DIV × V_multimetru / Vbat`). Pune în `VBAT_NOMINAL` tensiunea la care faci restul calibrării (ex. 7,6 V). Fără divizor: `USE_VBAT 0`.
2. **Senzori** (`s`): robotul centrat într-un culoar de 30 cm trebuie să vadă ~`SIDE_TARGET_MM` (≈112) la stânga și la dreapta. Dacă nu, măsoară din nou `SIDE_SENSOR_OFFSET_MM`.
3. **Reductorul**: pune în `GEAR_RATIO` raportul scris pe motor (ex. 30, 50, 100) și în `ENC_PPR` impulsurile pe rotație ale encoderului (de obicei 7).
4. **Sensul motoarelor**: `m` trebuie să meargă înainte. Altfel schimbă `MOTOR_L_INVERT` / `MOTOR_R_INVERT`.
5. **Encoderele** (`e`, **cu roțile în aer**): fiecare roată se învârte pe rând, iar rezultatul e unul dintre:
   - `OK`;
   - `sens INVERS` → schimbă `ENC_L_INVERT` / `ENC_R_INVERT`;
   - `NICIUN impuls` → verifică alimentarea (3,3 V) și firele A/B;
   - `INVERSATE între ele` → encoderul stâng e pe pinii celui drept: schimbă `PIN_ENC_L_*` cu `PIN_ENC_R_*`.

   La final comanda sugerează `MM_PER_S_AT_FULL`. Pune ~90 % din valoare (pe podea roțile merg mai încet decât în aer).
6. **Diametrul roții**: `s`, împinge robotul cu mâna fix 30 cm înainte, apoi iar `s`. Ambele encodere trebuie să crească cu ~300 mm. Dacă arată X mm: `WHEEL_DIAMETER_MM_nou = WHEEL_DIAMETER_MM × 300 / X`. Verifică apoi cu `m` că parcurge exact 30 cm.
7. **Bucla de viteză**: urmărește `out` și `v` în telemetrie (sau cu `m`).
   - roțile tremură, scot zgomot sau viteza oscilează → scade `SPEED_KP` (ex. 1.2 → 0.8), apoi `SPEED_KI`;
   - viteza rămâne mult sub țintă sau robotul nu urcă rampele/denivelările → crește `SPEED_KI`;
   - viteza țintă e mereu peste ce pot motoarele (`v` rămâne sub `out × MM_PER_S_AT_FULL` chiar și cu integratorul plin) → `MM_PER_S_AT_FULL` e prea mare, scade-l;
   - pentru comparație poți pune `SPEED_LOOP 0`.
8. **Virajele**: trimite `9` și `8` de câteva ori și măsoară unghiul (ex. cu o linie pe podea). Dacă virează sub unghi, mărește `TURN_SCALE` (ex. 1.00 → 1.04); dacă depășește, micșorează-l. Dacă 90° e bun dar 180° nu (sau invers), robotul alunecă la frânare: scade `TURN_KP` (frânează mai devreme) sau `V_TURN`. `0` (360°) e un test bun: greșelile se adună de 4 ori.
9. **Regulatorul de perete**: vezi mai jos, comun pentru ambele variante.
10. **Virajul în gol**: vezi mai jos.

`TURN_MS_90`, `TURN_MS_180` și `MOTOR_TRIM` nu se folosesc cu encodere.

### Fără encodere (`USE_ENCODERS 0`)

Totul e calibrat pe timp, deci **calibrează cu bateria la o tensiune cunoscută** și montează divizorul de baterie.

1. **Bateria** și **senzorii**: ca la pașii 1–2 de mai sus.
2. **Sensul motoarelor**: `m` trebuie să meargă înainte. Altfel schimbă `MOTOR_*_INVERT`.
3. **Mers drept** (`MOTOR_TRIM`): pune robotul pe o zonă deschisă, fără pereți lângă el, și trimite `m`. Dacă trage la stânga, mărește `MOTOR_TRIM` (ex. 0.02 → 0.04); dacă trage la dreapta, micșorează-l (poate fi negativ). E relativ la viteză (0.03 = roata stângă +3 %, dreapta −3 %), deci merge la fel la orice viteză și în marșarier.
4. **Distanța**: `m` (tot în zonă deschisă) trebuie să parcurgă exact 30 cm. Dacă face X mm, `MM_PER_S_AT_FULL_nou = MM_PER_S_AT_FULL × 300 / X`.
5. **Virajul pe 90°**: trimite `9` de câteva ori și măsoară unghiul. Ajustează `TURN_MS_90` până face fix 90° (viraj prea mic → mai mulți ms).
6. **Virajul pe 180°**: la fel cu `8` și `TURN_MS_180`. Ai nevoie de ambele: rampa de accelerație și alunecarea la frânare fac ca 180° să NU dureze dublul lui 90°. Celelalte unghiuri sunt interpolate pe dreapta dintre aceste două puncte. `0` (360°) e doar verificare.
7. **Regulatorul de perete** și **virajul în gol**: vezi mai jos. **Dacă schimbi `V_TURN` sau `ACCEL_PER_S`, refă pașii 5–6.**

Verificare: descarcă bateria cu ~0,5 V și repetă `9`/`8`. Cu divizorul montat, unghiurile ar trebui să rămână aproape la fel.

### Regulatorul de perete (ambele variante)

Comanda de direcție e `steer = KP_LAT·eroare_laterală + KP_SLOPE·pantă`, limitată la `±MAX_STEER` (`steer > 0` = virează la dreapta).

- **Eroarea laterală**: cu doi pereți = `(R − L)/2` (se centrează); cu un singur perete = diferența față de `SIDE_TARGET_MM`.
- **Panta** = cât s-a schimbat eroarea laterală pe mm parcurs (≈ tangenta unghiului față de perete; 0,1 ≈ 6°). Fără senzori diagonali, ea ține locul corecției de unghi și al termenului derivativ. Se calculează doar la citiri ToF noi și doar după minimum `SLOPE_MIN_MM` de mers, apoi e filtrată cu `SLOPE_FILT` și limitată la ±0,6. Pentru că e pe distanță, nu pe timp, se comportă la fel la orice viteză. Se resetează când se schimbă pereții văzuți (apare/dispare un perete) și după fiecare pivot.
- **Fără niciun perete**: cu encodere robotul ține direcția (`K_ENC_HOLD`), fără encodere merge drept.

Procedură:
1. Pornește cu `V_CRUISE = 0.5` într-un culoar drept, lung. Pune robotul puțin descentrat și puțin strâmb.
2. Robotul oscilează (șerpuiește) → crește `KP_SLOPE` (amortizează) sau scade `KP_LAT`.
3. Robotul se corectează leneș sau se lipește de un perete → crește `KP_LAT`.
4. Robotul revine la centru, dar trece de el și apoi se întoarce lent → crește `KP_SLOPE`. Dacă reacționează nervos, în smucituri → scade `KP_SLOPE` sau `SLOPE_FILT`, ori crește `SLOPE_MIN_MM` (pantă calculată pe o bază mai lungă, mai puțin zgomotoasă).
5. Robotul intră strâmb după viraje → la început panta e 0 (abia se resetase); verifică întâi virajele (`TURN_SCALE`), apoi crește puțin `KP_SLOPE`.
6. Pe porțiunile fără pereți deviază (cu encodere) → crește `K_ENC_HOLD`; oscilează → scade-l.
7. Urcă viteza treptat (`V_CRUISE` până la ~0.8–0.9) și refă pașii 2–4.
8. Robotul încetinește singur în corecții mari (`v × (1 − 0,6·|steer|)`), deci dacă stă des la `MAX_STEER`, regulatorul e prea agresiv.

Cu doar 3 senzori, unghiul se vede abia după ce robotul a mers câțiva mm, deci senzorii L/R trebuie montați exact la 90° și fără joc. Un senzor strâmb cu 5° dă o pantă falsă când robotul merge perfect drept.

### Virajul în gol

Când peretele de pe partea mâinii dispare, robotul mai merge `ADVANCE_TO_TURN_MM` (calculat ca să ajungă cu axa roților în centrul golului) și pivotează.

- După viraj atinge **colțul interior** → avansul e prea scurt: scade `OPEN_DETECT_LAG_MM` (crește avansul).
- După viraj atinge **peretele opus** → avansul e prea lung: crește `OPEN_DETECT_LAG_MM`.
- După viraj „sare” peste un gol imediat următor → crește `ENTER_AFTER_TURN_MM` puțin; dacă ratează un gol aflat imediat după viraj, scade-l.
- Se oprește prea departe / prea aproape de peretele din față înainte de pivot → verifică `FRONT_SENSOR_X_MM` (determină `FRONT_TURN_MM`).

### Mod de test

Pentru antrenament fără modulul IR: `TEST_START_BUTTON 1` (ții BOOT 1 s → pornește în 2 s). **La competiție pune înapoi 0!**

## Referință: toți parametrii din `config.h`

Valorile marcate **[CAL]** trebuie ajustate pe robotul real. Restul au valori bune implicite.

### 1. Pini (ESP-WROOM-32D)

Evită GPIO 6–11 (flash) și 12 (strapping). GPIO 34–39 sunt doar intrare și nu au pull-up intern.

| Parametru | Implicit | Descriere |
|---|---|---|
| `PIN_SDA` / `PIN_SCL` | 21 / 22 | I2C comun pentru toți senzorii ToF |
| `I2C_FREQ` | 400000 | frecvența I2C (400 kHz) |
| `PIN_XSHUT[]` | 16, 18, 23 | XSHUT pentru L, F, R (ordinea senzorilor din tabele) |
| `TOF_ADDR[]` | 0x10, 0x12, 0x14 | adresele I2C noi, în format 8-bit cum le cere librăria STM32duino (implicit senzorul are 0x52) |
| `PIN_ML_IN1/IN2`, `PIN_MR_IN1/IN2` | 25/26, 27/14 | intrările DRV8833 (2 pini PWM pe motor) |
| `MOTOR_L_INVERT` **[CAL]** | 0 | 1 dacă motorul stâng merge invers |
| `MOTOR_R_INVERT` **[CAL]** | 1 | motoarele sunt montate în oglindă, deci unul e de obicei inversat |
| `PIN_START` | 34 | modul Start/Stop: 0 la pornire, 1 = START, 0 = STOP |
| `PIN_KILL` | 35 | 1 la pornire, 0 = STOP (opțional) |
| `USE_KILL_PIN` | 1 | 0 dacă nu ai cablat pinul Kill |
| `USE_ENCODERS` | 1 | 0 = fără encodere (mers pe timp + ToF) |
| `PIN_ENC_L_A/B`, `PIN_ENC_R_A/B` | 32/33, 4/13 | fazele A/B ale encoderelor (pull-up intern activat) |
| `ENC_L_INVERT` / `ENC_R_INVERT` **[CAL]** | 0 / 1 | sensul de numărare; comanda `e` îți spune dacă trebuie schimbat |
| `USE_VBAT` | 1 | 0 dacă nu ai montat divizorul de baterie |
| `PIN_VBAT` | 36 | GPIO36 (VP), doar intrare, ADC1 |
| `VBAT_DIV` **[CAL]** | (100+47)/47 | raportul divizorului; 2S plin (8,4 V) → ~2,7 V pe pin |
| `VBAT_NOMINAL` **[CAL]** | 7.6 | tensiunea la care ai calibrat vitezele și virajele; PWM-ul se înmulțește cu `VBAT_NOMINAL / Vbat` (limitat la 0,80–1,25; în afara plajei 5,5–9 V compensarea e oprită) |
| `PIN_LED` | 2 | LED-ul de pe placă |
| `PIN_BUTTON` | 0 | butonul BOOT: apăsare scurtă = schimbă mâna |
| `TEST_START_BUTTON` | 0 | 1 = pornire din BOOT (ținut 1 s) în loc de modulul IR. **0 la competiție!** |

### 2. Geometrie [CAL – măsoară pe robot, în mm]

| Parametru | Implicit | Descriere |
|---|---|---|
| `CELL_MM` | 300 | distanța dintre pereți (din regulament) |
| `SIDE_SENSOR_OFFSET_MM` | 38 | de la axa longitudinală a robotului la fața senzorului L/R |
| `SIDE_SENSOR_X_MM` | 30 | cât de în fața axei roților sunt senzorii L/R |
| `FRONT_SENSOR_X_MM` | 45 | cât de în fața axei roților e senzorul F |
| `WHEEL_DIAMETER_MM` | 34 | diametrul roții; corectează-l din pasul „Diametrul roții” |
| `WHEEL_BASE_MM` | 78 | distanța dintre centrele roților (contează la pivot cu encodere) |
| `ENC_PPR` | 7 | impulsuri pe rotație ale motorului (înainte de reductor) |
| `GEAR_RATIO` | 50 | raportul reductorului, scris pe motor |
| `ENC_COUNTS_PER_REV` | calculat | `ENC_PPR × 4 × GEAR_RATIO` (×4 = decodare în cuadratură pe ambele fronturi ale A și B) |

### 3. Praguri derivate din geometrie (de obicei nu le schimbi)

| Parametru | Valoare | Descriere |
|---|---|---|
| `SIDE_TARGET_MM` | `CELL/2 − SIDE_SENSOR_OFFSET` ≈ 112 | distanța laterală când robotul e centrat |
| `WALL_PRESENT_MM` | `SIDE_TARGET + 75` | sub această distanță există perete lateral |
| `OPEN_MM` | `SIDE_TARGET + 115` | peste această distanță e deschidere laterală (între cele două praguri = histerezis) |
| `FRONT_TURN_MM` | `CELL/2 − FRONT_SENSOR_X` | distanța frontală la care axa roților e în centrul celulei (aici se pivotează) |
| `FRONT_SLOW_MM` | 420 | de aici începe frânarea spre un perete frontal |
| `OPEN_DETECT_LAG_MM` | 25 | conul ToF (~25°) vede deschiderea puțin mai târziu decât e de fapt |
| `ADVANCE_TO_TURN_MM` | `CELL/2 + SIDE_SENSOR_X − LAG` | cât avansezi după detectarea golului ca să ajungi în centrul lui |
| `ENTER_AFTER_TURN_MM` | `CELL/2 − SIDE_SENSOR_X + 20` | cât mergi drept după un viraj spre mână înainte să „armezi” iar detecția |
| `ENTER_SHORT_MM` | 50 | avansul după un viraj spre cealaltă parte sau după o întoarcere |

### 4. Viteze (fracțiuni 0..1) [CAL]

| Parametru | Implicit | Descriere |
|---|---|---|
| `MOTOR_MAX_DUTY` | 0.85 | limitează tensiunea efectivă pe motor (ex. N20 de 6 V pe LiPo 2S → ~0,8) |
| `MOTOR_DEADBAND` | 0.07 | PWM-ul minim la care N20 începe să se învârtă; comanda e mapată peste el |
| `V_CRUISE` | 0.80 | viteza pe drept |
| `V_MIN` | 0.28 | viteza minimă când te apropii de un perete frontal |
| `V_ADVANCE` | 0.55 | viteza în avansul spre viraj și la intrarea în culoar |
| `V_TURN` | 0.60 | viteza roților la pivotare |
| `V_TURN_MIN` | 0.25 | viteza minimă la finalul pivotării (cu encodere) |
| `V_EXIT` | 0.70 | viteza după detectarea ieșirii |
| `ACCEL_PER_S` | 7.0 | limita de accelerație (unități de comandă pe secundă; 7 = de la 0 la 0,8 în ~115 ms). Mai mic dacă roțile patinează la pornire |
| `PWM_FREQ` / `PWM_BITS` | 20000 / 10 | 20 kHz (nu se aude), rezoluție 10 biți |

### 5. Regulatoare [CAL]

| Parametru | Implicit | Descriere |
|---|---|---|
| `KP_LAT` | 0.0045 | per mm de eroare laterală |
| `KP_SLOPE` | 0.50 | per unitate de pantă (mm de eroare laterală / mm parcurs); corecția de unghi și amortizarea |
| `SLOPE_MIN_MM` | 8 | distanța minimă parcursă între două calcule de pantă (mai mare = mai puțin zgomot, reacție mai lentă) |
| `SLOPE_FILT` | 0.5 | filtrul pantei (0..1; mai mare = reacție mai rapidă, mai zgomotoasă) |
| `K_ENC_HOLD` | 0.0080 | ține direcția pe encodere când nu sunt pereți (per mm de diferență între roți) |
| `MAX_STEER` | 0.45 | limita comenzii de direcție |
| `TURN_KP` | 0.012 | decelerarea la finalul pivotării cu encodere (per mm de arc rămas); mai mic = frânează mai devreme |
| `TURN_SCALE` | 1.00 | >1 dacă virează sub unghi, <1 dacă depășește (cu encodere) |
| `SPEED_LOOP` | 1 | buclă PI de viteză pe roți (cu encodere); 0 = PWM direct |
| `SPEED_KP` | 1.2 | proporțional, per eroare de viteză normalizată (`eroare / MM_PER_S_AT_FULL`) |
| `SPEED_KI` | 6.0 | integral, per secundă |
| `SPEED_I_MAX` | 0.35 | limita integratorului (anti-windup, ex. când roata e blocată de un perete) |
| `SPEED_FILT` | 0.35 | filtrul pe viteza măsurată (0..1; mai mare = răspuns mai rapid, dar mai zgomotos) |
| `TURN_MS_90` | 190 | fără encodere: ms de rotire la `V_TURN` pentru exact 90° |
| `TURN_MS_180` | 335 | fără encodere: ms pentru exact 180° |
| `MM_PER_S_AT_FULL` | 600 | viteza la comanda 1.0. Fără encodere: odometria estimată. Cu encodere: scara buclei de viteză (o sugerează comanda `e`) |
| `MOTOR_TRIM` | 0.00 | fără encodere: corecție de mers drept, relativă la viteză (+ dacă trage la stânga pe gol) |

### 6. Senzori ToF

| Parametru | Implicit | Descriere |
|---|---|---|
| `TOF_TIMING_BUDGET_US` | 20000 | 20 ms pe măsurătoare → ~50 Hz pe senzor. Mai mic = mai rapid dar mai zgomotos și cu bătaie mai mică |
| `TOF_FAR_MM` | 2000 | valoarea raportată când senzorul nu vede nimic |
| `TOF_STALE_MS` | 150 | o citire mai veche e considerată senzor blocat și e tratată ca perete la 60 mm (conservator: nu declanșează viraj sau ieșire) |

Din fiecare măsurătoare se ia cea mai apropiată țintă validă. Sub ~25 mm senzorul dă erori, așa că dacă tocmai fusese sub 80 mm (în ultimele 200 ms), o citire invalidă nu e raportată ca „gol”.

### 7. Ieșire, adversar, blocare

| Parametru | Implicit | Descriere |
|---|---|---|
| `EXIT_FAR_MM` | 900 | ieșire = L, F și R văd toți peste această distanță. După `EXIT_RUNOUT_MM` se reverifică cu 80 % din prag; dacă se văd pereți, robotul continuă. Crește-l dacă labirintul are intersecții cu culoare lungi în toate direcțiile (alarme false); scade-l dacă afară, după poartă, sunt obiecte apropiate (ieșire ratată) |
| `EXIT_CONFIRM_MM` | 100 | cât trebuie să meargă robotul cu condiția îndeplinită (pe distanță, nu pe timp, ca să nu „iasă” stând pe loc) |
| `EXIT_MIN_TIME_MS` | 2500 | în primele secunde nu poți „ieși” (ești încă la intrare) |
| `EXIT_RUNOUT_MM` | 180 | cât mai mergi după ieșire ca să fii complet afară |
| `OPP_MOTION_MM` | 45 | adversar = obstacolul frontal (sub 600 mm) s-a apropiat cu atât PESTE cât am mers noi, pe ultimele 8 citiri |
| `OPP_JUMP_MM` | 150 | adversar = distanța frontală scade brusc cu atât (robot venit din lateral) |
| `OPP_MEMORY_MS` | 700 | cât timp ținem minte că am văzut adversarul |
| `OPP_WAIT_MS` | 2200 | cât așteptăm adversarul să plece înainte să decidem normal |
| `OPP_BACKOFF_MM` | 60 | dacă e mai aproape de atât, dăm puțin înapoi |
| `STUCK_WINDOW_MS` | 800 | la câte ms se verifică blocarea |
| `STUCK_MIN_MOVE_MM` | 12 | blocat = motoarele trag (comanda > 0,25) dar robotul s-a mișcat sub atât. Cu encodere, și dacă roțile au mers > 60 mm dar peretele frontal (sub 400 mm) nu s-a apropiat (patinare) |

### 8. Bucla de control / debug

| Parametru | Implicit | Descriere |
|---|---|---|
| `CONTROL_PERIOD_MS` | 5 | perioada buclei de control (200 Hz). Dacă o schimbi, reverifică regulatoarele |
| `DEBUG_SERIAL` | 1 | telemetrie pe Serial la 10 Hz (115200) |

### Constante fixe din `MazeRunner.ino`

Câteva valori mai rar de schimbat sunt direct în cod:

| Unde | Valoare | Ce face |
|---|---|---|
| `driveDistance` | `V_MIN + rest·0.006` | decelerarea spre finalul unei distanțe |
| `alignFront` | max 180 ms, toleranță 6 mm, câștig 0.006, viteză 0,18–0,30 | ajustarea fină a distanței față de peretele frontal înainte de pivot |
| `pivot` (cu encodere) | timeout 1500 ms | siguranță dacă un encoder nu numără |
| `stuckCheck` | 250 ms înapoi la −0,45 cu viraj 0,25 | manevra de deblocare (spre partea mai liberă) |
| `handOpeningEvent` | 15 ms | cât trebuie să dureze golul ca să fie luat în seamă |
| `waitForClear` | `FRONT_TURN_MM + 120`, 120 ms | când considerăm culoarul eliberat de adversar |
| `frontDecision` | 30 ms frână + 45 ms pauză | citiri proaspete cu robotul oprit înainte de decizie |
| `encoderTest` | comanda 0,6, măsoară între 300 și 600 ms | testul `e` |
| `updateVbat` | la 20 ms, filtru 0,1 (~200 ms) | filtrarea tensiunii bateriei |
| `safety` / `loop` | 3 citiri consecutive | filtrarea semnalelor START/STOP |
| `doExit` | 80 ms frână + 60 ms pauză, prag 80 % din `EXIT_FAR_MM` | reverificarea ieșirii cu robotul oprit |
| `wallSteer` | pantă limitată la ±0,6 | protecție la salturi ale distanței laterale |

## Note de implementare

- **Motoare**: DRV8833 în mod „slow decay” (un pin HIGH, celălalt PWM inversat): viteza e aproape liniară cu duty-ul și frânarea e activă. Frâna = ambii pini HIGH; oprirea finală = ambii LOW (roată liberă).
- **Encodere**: întrerupere pe ambele fronturi ale A și B (×4), decodate cu un tabel de tranziții. Pinii se citesc direct din registrul GPIO, pentru că `digitalRead()` e lent și nu e garantat în IRAM pe core 3.x.
- **Bucla de viteză**: feedforward = comanda însăși, plus PI pe eroarea dintre comandă și `v / MM_PER_S_AT_FULL`. La frână sau comandă ~0 integratorul se resetează. Înainte de start și de fiecare comandă Serial estimarea vitezei se resetează (robotul poate fi mutat cu mâna între timp).
- **Ținerea direcției** fără pereți: referința e diferența dintre encodere când s-a văzut ultima oară un perete. După pivot sau deblocare referința se ia de la zero, altfel robotul ar încerca să revină la direcția veche.
- **Odometria fără encodere**: model (comandă × timp), iar dacă se vede un perete frontal, scăderea distanței F (dacă e plauzibilă) e folosită în locul modelului.
- **Detecția adversarului** e oprită în timpul pivotării (rotirea schimbă brusc distanța frontală).
- **Start/Stop** (Anexa 2): dacă START e 1 la pornire, robotul se blochează cu eroare. După STOP motoarele frânează, apoi rămân libere, iar robotul stă blocat până la repornirea alimentării.
- Senzorii ToF se citesc într-un task separat pe **core 0**; bucla de control rulează la 200 Hz pe core 1 și lucrează pe o copie a ultimelor citiri.

## Checklist omologare
- Dimensiune ≤ 10 × 10 cm, numărul de concurs lipit pe carcasă.
- Pornește cu LED-ul verde al modulului **stins**; arbitrul programează modulul; START pornește, STOP oprește și robotul rămâne blocat până la repornire.
- Mâna aleasă corect pentru intrarea pe care ești pus.

# MazeRunner – Robochallenge Maze (varianta simplă)

Firmware ESP32 pentru proba **Maze** (regulament: robochallenge.ro/regulation/maze).

Varianta asta e făcută să meargă cu **foarte puțină testare**: fără encodere, viteze mici, iar tot ce depinde de robotul fizic (mâna, sensul motoarelor, ordinea senzorilor, timpii de viraj, viteza) se reglează **din Serial Monitor, fără reîncărcarea codului**, și se salvează în flash.

## Regulament, pe scurt

| Regulă | Ce face firmware-ul |
|---|---|
| Robot max 10 × 10 cm; culoare de 30 cm, pereți de 15 cm | praguri calculate din `CELL_MM = 300` |
| 2 roboți pornesc din intrări simetrice; câștigă primul care iese | urmărire de perete + detecție ieșire; așteaptă dacă obstacolul din față se mișcă |
| Ieși doar prin zona de ieșire | alegi mâna corectă înainte de meci |
| Modul Start/Stop IR obligatoriu: START=0 la pornire, 1 = start, 0 = stop | implementat; dacă START e 1 la pornire, modulul e ignorat până la repornire |

## Strategia

Urmărire de perete cu mâna stângă sau dreaptă:
- pe culoar se centrează între pereți (sau pe un singur perete), încetinind lângă peretele din față;
- **gol pe partea mâinii** → avansează până în centrul golului → pivotează 90° → intră în noul culoar;
- **perete în față** → mâna liberă? → cealaltă parte liberă? → altfel întoarcere 180°;
- dacă obstacolul din față se mișcă (adversarul), așteaptă până la 2 s;
- **ieșire**: toți senzorii văd peste 90 cm timp de 0,4 s → mai merge 18 cm și se oprește (dacă vede iar pereți, continuă).

➡️ **Înainte de fiecare meci alege mâna** după intrarea pe care ești pus: apăsare scurtă pe BOOT, sau `l` / `r` în Serial. Se salvează în flash.

## Cablaj (după „Maze_ESP32_Componente_si_Conexiuni.docx”)

| Semnal | GPIO |
|---|---|
| I2C SDA / SCL (toți senzorii VL53L0X) | 21 / 22 |
| XSHUT senzori stânga / față / dreapta | 25 / 26 / 27 |
| DRV8835 (mod PHASE/ENABLE, MODE la 3V3) motor stâng PH / EN | 18 / 19 |
| DRV8835 motor drept PH / EN | 23 / 32 |
| Encodere stânga / dreapta (cablate, nefolosite) | 34, 35 / 36, 39 |
| LED / buton | 2 / 0 (BOOT) |
| Liber (pentru modulul IR, mai târziu) | 33 |

Alimentare: LiPo 2S → step-down 6 V pentru motoare (VIN pe DRV8835), step-down 5 V pentru ESP32.

## Pornire

Momentan **fără telecomandă IR** (`USE_START_MODULE 0`): robotul pornește singur la **20 s după alimentare**. LED-ul clipește rapid cât numără; apoi rămâne aprins în cursă.
- Apăsare scurtă pe BOOT = schimbă mâna **și reîncepe numărătoarea** de 20 s.
- Orice comandă trimisă din Serial **anulează** pornirea automată (pentru calibrare); după aceea pornești cu `g`.
- În cursă: BOOT sau orice tastă din Serial oprește robotul (reset cu EN pentru o nouă încercare).

Pentru competiție (modul IR obligatoriu): conectează START la GPIO33 și pune `USE_START_MODULE 1` în `config.h`.

## Instalare

**VS Code + PlatformIO:** deschide folderul în VS Code. Din bara de jos: ✓ Build, → Upload, 🔌 Serial Monitor (115200). Biblioteca VL53L0X (Pololu) se instalează singură.

**Arduino IDE:** placa *ESP32 Dev Module* (esp32 by Espressif 3.x), biblioteca **VL53L0X by Pololu**.

## Testare în 15 minute

Încarcă codul **o singură dată**, deschide Serial Monitor (115200) și trimite orice tastă în primele 20 s, ca robotul să nu pornească singur. Apoi fă pașii în ordine. Fiecare reglaj se salvează imediat. `h` afișează lista de comenzi. Dacă robotul se oprește cu „[STOP]”, apasă EN (reset): parametrii rămân salvați.

1. **Pornire (1 min).** Trebuie să apară `[ToF] GPIO.. OK` de 3 ori. Dacă un senzor dă eroare, verifică firele; codul reîncearcă singur la fiecare secundă.
2. **Senzori (2 min).** Trimite `s` și ține mâna pe rând în fața senzorului stâng, din față și din dreapta. Valoarea care scade trebuie să fie în coloana corectă (`L`, `F`, `R`); peste ~1,2 m senzorii VL53L0X arată 2000 („nimic”). Dacă nu, trimite `p` (trece la următoarea ordine) până e corect.
3. **Motoare (2 min).** Ridică robotul. `1` = roata stângă trebuie să meargă **înainte**; dacă merge înapoi, `a`. `2` = roata dreaptă; dacă merge înapoi, `b`.
4. **Viraj 90° (3 min).** Pune robotul pe podea. `9` = pivotează spre stânga. A rotit prea puțin: `+`; prea mult: `-` (câte 10 ms). Repetă `9` până iese ~90°.
5. **Viraj 180° (2 min).** La fel cu `8`, apoi `+` / `-`.
6. **Viteză (1 min).** Robotul cu fața la un perete aflat la 60–110 cm, trimite `v`. Măsoară și salvează viteza reală (contează pentru unde virează la goluri).
7. **Cursă de test (4 min).** Pune robotul în labirint, alege mâna (`l` / `r`), trimite `g`. Pornește după 2 s; îl oprești cu BOOT sau orice tastă. Urmărește în Serial starea (`FOLLOW`, `ADVANCE`, `PIVOT`, `DECIDE`...).

Dacă mai ai timp:
- robotul trage într-o parte pe porțiunile fără pereți → `q` (corecție spre stânga) / `w` (spre dreapta);
- virează prea devreme sau prea târziu la goluri → refă `v` sau, în cod, ajustează `OPEN_DETECT_LAG_MM` din `config.h`;
- `f` merge o singură celulă (30 cm), util ca să vezi centrarea pe culoar.

**La competiție:** dacă modulul IR e montat, pune `USE_START_MODULE 1` și START pe GPIO33.

## Parametri (`config.h`)

Cei salvați în flash (`t90`, `t180`, viteză, trim, mână, sensuri, ordinea senzorilor) au valorile implicite `DEF_*` și se schimbă din Serial (`z` revine la implicite). Restul se schimbă în `config.h`:

| Parametru | Implicit | Ce face |
|---|---|---|
| `V_RUN` / `V_SLOW` / `V_ADVANCE` / `V_TURN` | 0.45 / 0.25 / 0.35 / 0.45 | vitezele (fracție din PWM). Dacă le schimbi, refă virajele (`9`, `8`) și viteza (`v`) |
| `MOTOR_DEADBAND` | 0.10 | PWM-ul minim la care motoarele încep să se miște |
| `KP_LAT` / `KD_LAT` / `MAX_STEER` | 0.004 / 0.0015 / 0.30 | centrarea pe culoar. Oscilează: scade `KP_LAT` sau crește `KD_LAT`. Se lovește de pereți: crește `KP_LAT` |
| `SIDE_SENSOR_OFFSET_MM` | 38 | distanța de la centrul robotului la fața senzorului lateral |
| `SIDE_SENSOR_X_MM` / `FRONT_SENSOR_X_MM` | 30 / 45 | poziția senzorilor față de axul roților |
| `EXIT_FAR_MM` | 900 | pragul pentru „sunt afară” |
| `OPP_WAIT_MS` | 2000 | cât așteaptă după un adversar |

## Checklist omologare

- Dimensiune ≤ 10 × 10 cm, numărul de concurs lipit pe carcasă.
- Pornește cu LED-ul verde al modulului **stins**; arbitrul programează modulul; START pornește, STOP oprește și robotul rămâne blocat până la repornire.
- Mâna aleasă corect pentru intrarea pe care ești pus.

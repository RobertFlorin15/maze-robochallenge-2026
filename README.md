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

## Cablaj (după „Maze Electronica Schema Finala.pdf”)

| Semnal | GPIO |
|---|---|
| I2C SDA / SCL (toți ToF) | 21 / 22 |
| XSHUT senzori (sus / mijloc / jos în schemă) | 15 / 2 / 23 |
| DRV8835 motor stâng IN1 / IN2 | 25 / 26 |
| DRV8835 motor drept IN1 / IN2 | 27 / 14 |
| Start module START / KILL | 18 / 19 (pull-down intern) |
| Buton | 0 (BOOT) |

Encoderele (32/33, 34/35) nu sunt folosite. LED-ul de pe GPIO2 e oprit, pentru că GPIO2 e XSHUT-ul unui senzor.

## Instalare

**VS Code + PlatformIO:** deschide folderul în VS Code. Din bara de jos: ✓ Build, → Upload, 🔌 Serial Monitor (115200). Biblioteca VL53L3CX se instalează singură.

**Arduino IDE:** placa *ESP32 Dev Module* (esp32 by Espressif 3.x), biblioteca **STM32duino VL53L3CX**.

⚠️ Dacă upload-ul dă „Failed to connect”: ține apăsat **BOOT** cât timp începe încărcarea. Senzorul de pe GPIO2 trage pinul în sus și încurcă intrarea automată în modul de programare.

## Testare în 15 minute

Încarcă codul **o singură dată**, deschide Serial Monitor (115200) și fă pașii în ordine. Fiecare reglaj se salvează imediat. `h` afișează lista de comenzi. Dacă robotul se oprește cu „[STOP]”, apasă EN (reset): parametrii rămân salvați.

1. **Pornire (1 min).** Trebuie să apară `[ToF] GPIO.. OK` de 3 ori. Dacă un senzor dă eroare, verifică firele; codul reîncearcă singur la fiecare secundă.
2. **Senzori (2 min).** Trimite `s` și ține mâna pe rând în fața senzorului stâng, din față și din dreapta. Valoarea care scade trebuie să fie în coloana corectă (`L`, `F`, `R`). Dacă nu, trimite `p` (trece la următoarea ordine) până e corect.
3. **Motoare (2 min).** Ridică robotul. `1` = roata stângă trebuie să meargă **înainte**; dacă merge înapoi, `a`. `2` = roata dreaptă; dacă merge înapoi, `b`.
4. **Viraj 90° (3 min).** Pune robotul pe podea. `9` = pivotează spre stânga. A rotit prea puțin: `+`; prea mult: `-` (câte 10 ms). Repetă `9` până iese ~90°.
5. **Viraj 180° (2 min).** La fel cu `8`, apoi `+` / `-`.
6. **Viteză (1 min).** Robotul cu fața la un perete aflat la 60–120 cm, trimite `v`. Măsoară și salvează viteza reală (contează pentru unde virează la goluri).
7. **Cursă de test (4 min).** Pune robotul în labirint, alege mâna (`l` / `r`), trimite `g`. Pornește după 2 s; îl oprești cu BOOT sau orice tastă. Urmărește în Serial starea (`FOLLOW`, `ADVANCE`, `PIVOT`, `DECIDE`...).

Dacă mai ai timp:
- robotul trage într-o parte pe porțiunile fără pereți → `q` (corecție spre stânga) / `w` (spre dreapta);
- virează prea devreme sau prea târziu la goluri → refă `v` sau, în cod, ajustează `OPEN_DETECT_LAG_MM` din `config.h`;
- `f` merge o singură celulă (30 cm), util ca să vezi centrarea pe culoar.

**La competiție:** `TEST_START_BUTTON` trebuie să fie `0` (implicit). Robotul pornește doar din modulul IR.

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

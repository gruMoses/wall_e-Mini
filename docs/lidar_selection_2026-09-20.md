# LiDAR selection research (Grok 4.6 web research, 2026-09-20)

Session 01a0c109-6dbe-74c1-bfc8-a3929c2b7a68. Prices and specs are as found on that date; verify before ordering.

I'll research the 2D/low-cost 3D LiDAR field against WALL-E Mini's gate, sun, rain, and Pi 5 constraints, citing a URL for every spec and price. Starting with datasheets, current street prices, and Python-driver reports.Workspace is a scratchpad, so I'll keep pulling official datasheets, current US prices, and Python-driver reports for the shortlist.A few official datasheets and street prices still missing. Pulling those plus user reports on sun-blindness and water ingress.**Pick: Slamtec RPLIDAR S2L.** It is the only hobby-priced 2D unit that is simultaneously DTOF, 360°, 5 cm minimum range, **80 klux**, **IP65**, UART, and 5 V. STL-19P is better at the gate and half the price, but 60 klux is not Texas noon and IP5X is not rain.

Texas noon is ~100 klux (that is the outdoor test condition Livox and Unitree publish). 30–40 klux units go blind in sun. 60 klux is morning/overcast, not high noon.

---

## Ranked by job fit

| Band | Unit | Buy | Why / why not |
|---|---|---|---|
| **Primary** | **RPLIDAR S2L** | **$299** [DFRobot](https://www.dfrobot.com/product-2617.html) / [Seeed](https://www.seeedstudio.com/RPLiDAR-S2L-360-Degree-Laser-Range-Scanner-18M-Range-p-5492.html) | IP65 + 80 klux + DTOF + UART. ±30 mm is usable at a gate, not as tight as STL-19P. |
| **Budget** | **LDROBOT STL-19P (D500 kit)** | **$79** [Amazon](https://www.amazon.com/LDROBOT-Outdoor-Navigation-Scanning-Support/dp/B0DDKXQ23R) / $119 [DFRobot](https://www.dfrobot.com/product-2610.html) | Best close-range (±10 mm @ 3–50 cm). 60 klux, **IP5X (dust only)**. Garage it. |
| **If truly weatherproof** | **S2L** (same) | $299 | Cheapest **genuine IP65** 360° DTOF I found. Next is YDLIDAR TG15 at $317. |
| Mid, no rain | STL-27L | $168 [Waveshare](https://www.waveshare.com/DTOF-LIDAR-STL27L.htm) | 25 m, 0.167°, ±15 mm close, 60 klux. No IP published. |
| 3D if you stretch | Unitree L2 | $419 [unitree.com/L2](https://www.unitree.com/L2/) | 100 klux, UART **or** UDP, 5 cm min, 2 cm. **IP54** — Unitree says add a water cover. |
| 3D overkill | Livox Mid-360 | ~$600–900 [US reseller](https://sonnyrobotics.com/products/livox-mid-360); DJI HK [HKD $4,499](https://store.dji.com/hk/product/livox-mid-360) | IP67, 100 klux, 0.1 m min. Ethernet, C++/ROS — not a Pi serial driver. |

**Cheapest genuine IP65+ 360° 2D:** RPLIDAR S2L **$299**. TG15 is $316.92 [RobotShop](https://www.robotshop.com/products/ydlidar-tg15-360-laser-scanner). Nothing credible under $150 is IP65.

---

## Serious candidates

### Under ~$150

**LDROBOT STL-19P / D500 kit — budget pick**
- Price: $79 Amazon ([listing](https://www.amazon.com/LDROBOT-Outdoor-Navigation-Scanning-Support/dp/B0DDKXQ23R)); $119 DFRobot SEN0547 ([page](https://www.dfrobot.com/product-2610.html)); €80.41 [botnroll](https://www.botnroll.com/en/infrared/5864-stl-19p-lidar-dtof-with-usb-adapter-360-12m-radius-uart-d500-lidar-kit.html)
- Specs: DTOF, 0.03–12 m (80% white) / 0.03–8 m (4% black), 0.72° @ 10 Hz, 6–13 Hz scan (typ 10), 5 kHz sample, **60 klux**, **IP5X**, −10–45 °C, UART 230400, 5 V / 290 mA / 1.45 W, 45 g, brushless BLDC, 10 kh life. Accuracy ±10 mm @ 0.03–0.5 m, ±20 mm @ 0.5–2 m, ±30 mm @ 2–12 m. Shock: **not published**.
- Datasheet: [EN PDF](https://download.kamami.pl/p1188450-LDROBOT_STL-19P_Datasheet_EN_v1.0%281%29.pdf), [Waveshare D500 wiki](https://www.waveshare.net/wiki/D500_LiDAR_Kit)
- Python: community UART parsers — [halac123b LD19](https://github.com/halac123b/Visualize-data-from-Lidar-LD19_Matplotlib-Python) (same packet family), [pschatzmann STL](https://github.com/pschatzmann/LDROBOT-LIDAR-STL) (Arduino, protocol is documented). Official is C++/ROS2 ([ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git)). **Plain pyserial driver is easy.**
- Failures: IP5X is dust, not rain. 60 klux ≠ Texas noon. Open optical window. Fine for garage; not for being left outside.

**RPLIDAR C1 — skip for this robot**
- Price: $69 [DFRobot](https://www.dfrobot.com/product-2803.html)
- Specs: DTOF, 0.05–12 m (70% white) / 0.05–6 m (10% black), 0.72°, 8–12 Hz, 5 kHz, **40 klux**, **IP54**, ±30 mm, UART 460800, 110 g, −10–40 °C. Datasheet: [C1 PDF](https://d229kd5ey79jzj.cloudfront.net/3157/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf). Python: [rplidarc1](https://github.com/dsaadatmandi/rplidarc1) (pyserial).
- Why not: 40 klux is shade. IP54 is splash, not a hose. ±30 mm and 5 cm min are worse at a tight gate than STL-19P.

**LDROBOT LD19 — skip (superseded)**
- Waveshare **discontinued**, points at D500 ([product](https://www.waveshare.com/dtof-lidar-ld19.htm)). DTOF, 0.02–12 m, 4.5 kHz, 0.8° @ 10 Hz, **30 klux**, IP **not published**, 180 mA, 47 g, UART 230400, −10–40 °C. Datasheet: [DFRobot PDF](https://cdn.robotshop.com/media/D/Dfr/RB-Dfr-1273/pdf/dfrobot-dtof-ld19-laser-lidar-sensor-kit-12m-datasheet.pdf). 30 klux is indoor.

**LDROBOT LD06 — skip**
- DTOF, 0.02–12 m, 4.5 kHz, **30 klux**, **IPX4**, brushless, 180 mA, 42 g. Datasheet: [PDF](https://make.net.za/wp-content/datasheets/LDROBOT%20LD06%20Datasheet.pdf). IPX4 is splash. Python: [ldrobot-ld06-lidar-python-driver](https://github.com/drinking-code/ldrobot-ld06-lidar-python-driver). ~$70–100 street.

**YDLIDAR T-mini Plus — also-ran**
- Price: $79 [RobotShop Yahboom](https://www.robotshop.com/products/yahboom-yahboom-t-mini-plus-lidar-tof-ranging-12m-support-ros1-ros2) / $95 [RobotShop YDLIDAR](https://www.robotshop.com/products/ydlidar-t-mini-plus-2d-compact-lidar-sensor)
- ToF, 0.05–12 m (80%) / 0.05–4 m (10%), 4 kHz, 6–12 Hz, 0.54°, **60 klux**, IP **not published**, ±20 mm typ, UART 230400, 340 mA, 45 g, −10–45 °C. Python: official [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) (C++ with Python bindings) + ROS2. Same outdoor holes as STL-19P, worse sample rate.

**YDLIDAR X4 Pro — skip**
- $71 [Amazon](https://www.amazon.com/Triangular-Scanning-Obstacle-Avoidance-Navigation/dp/B0CQLWBT52). **Triangulation, belt, 1500 h life**, 0.12–10 m indoor, lighting 2k typ / 40k max, IP none. [Datasheet](https://static.generation-robots.com/media/YDLIDARX4PRODatasheet.pdf). Sun-blind, belt dies on a vibrating chassis.

**RPLIDAR A1M8 — skip**
- $99 [DFRobot](https://www.dfrobot.com/product-1125.html) / $103 [RobotShop](https://www.robotshop.com/products/rplidar-a2m12-360-laser-range-scanner) (A1 kit). Triangulation, belt, indoor, “no direct sunlight”, no IP. Python: [Skoltech rplidar](https://github.com/SkoltechRobotics/rplidar) (A1/A2 protocol only).

### ~$150–400

**RPLIDAR S2L — primary**
- Price: **$299** [DFRobot DFR1023](https://www.dfrobot.com/product-2617.html), [Seeed](https://www.seeedstudio.com/RPLiDAR-S2L-360-Degree-Laser-Range-Scanner-18M-Range-p-5492.html), $295 [youyeetoo](https://youyeetoo.com/products/slamtec-rplidar-s2l-lidar-sensor)
- DTOF, 0.05–18 m (90%) / 0.05–8 m (10%), **0.12°**, 10 Hz (8–15), **32 kHz**, ±30 mm, 13 mm res, **>80 klux**, **IP65**, −10–50 °C, UART 1 Mbps, 5 V, >2 W, 190 g, 77×77×39 mm, **brushless + OPTMAG (no belt), sealed cover**. Shock: **not published**.
- Specs: [DFRobot](https://www.dfrobot.com/product-2617.html), [Waveshare S2 wiki](https://www.waveshare.com/wiki/RPLIDAR_S2), [S2 datasheet PDF](https://grobotronics.com/images/SLAMTEC_rplidar_datasheet_S2_v2.0_en.pdf?1758179377278)
- Python: **[pyrplidarsdk](https://pypi.org/project/pyrplidarsdk/)** wraps official SDK (serial + UDP). Official C++: [Slamtec rplidar_sdk](https://github.com/Slamtec/rplidar_sdk). Do **not** use Skoltech `rplidar` — that is A1/A2 triangulation.
- Failures: one DE Amazon Vine review of a unit dead at 5 months ([WayPonDEV S2](https://www.amazon.de/WayPonDEV-Scanradius-Hindernisvermeidung-Navigation-Robotern/dp/B0B1JG527R)); another user reports outdoor sun, glass, and night lights working. Window will scratch; 80 klux is still short of 100 klux noon. 1 Mbps UART on Pi 5 is fine with a real USB-UART (not a fake CH340).

**RPLIDAR S2 (30 m) — if S2L is out of stock**
- $399 [DFRobot](https://www.dfrobot.com/product-2803.html) / $400 [Amazon](https://www.amazon.com/-/es/WayPonDEV-RPLIDAR-escaneo-obst%C3%A1culos-navegaci%C3%B3n/dp/B09XD4C52M). Same IP65 / 80 klux / 32 kHz; 0.05–30 m white / 10 m black. Same driver.

**LDROBOT STL-27L — best close-range in this band, not rainproof**
- $168 [Waveshare](https://www.waveshare.com/DTOF-LIDAR-STL27L.htm)
- DTOF, 0.03–25 m (80%) / 0.03–10 m (4%), **0.167°**, 10 Hz, **21.6 kHz**, ±15 mm @ 0.03–2 m, **60 klux**, IP **not published**, UART **921600**, 290 mA / 1.45 W, 46 g, −10–50 °C, brushless, glass-wall detection claimed. [Wiki](https://www.waveshare.com/wiki/DTOF_LIDAR_STL27L)
- Python: same LDROBOT packet family as LD19; PiLiDAR has a serial driver ([PiLiDAR](https://github.com/PiLiDAR/PiLiDAR)). Official ROS2.
- Failures: 60 klux; no IP; 921600 needs a decent USB-UART.

**YDLIDAR TG15 — IP65 + 100 klux, worse at the gate**
- $316.92 [RobotShop](https://www.robotshop.com/products/ydlidar-tg15-360-laser-scanner); ~€318 SOS
- ToF, 0.05–15 m, 20 kHz, 5–12 Hz, 0.09–0.22°, **100 klux**, **IP65**, ±60 mm (0.05–5 m) / ±40 mm (5–15 m), UART 512000, 400–480 mA, brushless, 0–50 °C. [SOS listing](https://www.soselectronic.com/it-sm/distance-measurement/ydlidar)
- Python: official [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK).
- Why not primary: ±60 mm at post range is sloppy for a gate a few inches wider than 0.8 m. Heavier current. S2L is cheaper and more accurate.

**YDLIDAR TG30** — same family, 0.05–30 m, IP65, 100 klux. Street ~€350–400. Overkill range, same ±60 mm close.

**RPLIDAR A2M12 — skip**
- $229 [DFRobot](https://www.dfrobot.com/product-1461.html) / $239 [RobotShop](https://www.robotshop.com/products/rplidar-a2m12-360-laser-range-scanner). Triangulation, 0.2–12 m, 16 kHz, 0.225°, **indoor / no direct sun**, no IP, brushless OPTMAG, 450–600 mA, 0–40 °C. [Datasheet](https://download.kamami.pl/p1188391-LD310_SLAMTEC_rplidar_datasheet_A2M12_v1.0_en.pdf).

**RPLIDAR S2E / S3** — S2E $469 Ethernet 12 V; S3 $549, 40 m, 80 klux, IP65. No reason to pay this for Mini.

**Unitree L1** — $249 [shop](https://shop.unitree.com/products/unitree-4d-lidar-l1) / [unitree.com/LiDAR](https://www.unitree.com/LiDAR/). 3D, 100 klux, **IP54**, UART 2 Mbps, 0.05 m, ±2 cm, 6 W / 12 V, 230 g. Manual: [L1 PDF](https://oss-global-cdn.unitree.com/Lidar/L1%20Quike%20Start%20Guide_v1.0.pdf). Official SDK is C++/ROS; Python is a UDP subscriber example ([unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk)). Not rain-rated.

**Unitree L2** — $419 [unitree.com/L2](https://www.unitree.com/L2/). 3D (has a 2D mode), 100 klux, **IP54**, 0.05 m, ≤2 cm, 64k pts/s, UART 4 Mbps **or** Ethernet UDP, 10 W / 12 V (13 W peak with heater). Manual: [L2 PDF](https://cdn-reichelt.de/documents/datenblatt/C700/UNITREE4DLIDARL2USERMANUAL.pdf). Unitree: *“When the application scenario requires water resistance, the L2 needs to be equipped with a water protection device.”* SDK: [unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2.git) (C++/ROS, UDP examples). Better sun than S2L, worse rain, more power, more driver work.

### Industrial (honest prices)

| Unit | Price | IP | klux | FOV | Notes |
|---|---|---|---|---|---|
| Hokuyo URG-04LX-UG01 | $750 sale [Acroname](https://acroname.com/store/r325-urg-04lx-ug01); MSRP ~$1,148 | none | 10 | 240°, 5.6 m | Indoor triangulation. Dies outdoors ([Acroname test](https://acroname.com/blog/hokuyo-laser-range-finder-comparison-0)). |
| Hokuyo UST-10LX | ~$1,200 [Acroname](https://acroname.com/store/r325-urg-04lx-ug01) / €1,365 [ROS Components](https://www.roscomponents.com/product/ust-10lx/) | IP65 | 10–15 | 270°, 10 m, 40 Hz, Ethernet | Still not Texas sun. |
| SICK TiM310 | ~$1,000–1,500 [meetsensor](https://meetsensor.com/products/german-sick-laser-scanner-sensor-model-tim310-1030000) / [Farnell](https://ie.farnell.com/c/sensors-transducers/sensors/optical-sensors/photoelectric-sensors?brand=sick&product-range=tim-series) | IP65–67 | **80** | 270°, 4 m | 4 m is too short for 8–12 m. IEC 60068 shock. |
| SICK TiM5xx | $2k–4k | IP67 | 80 | 270°, 25 m | Real industrial. Not hobby. |

**Livox Mid-360** — IP67, **100 klux**, 0.1–70 m (80%) / 40 m (10%), 200k pts/s, Ethernet, 6.5 W, 9–27 V, 265 g, −20–55 °C, ISO 16750-3 vibration. [Manual specs](https://manualspro.net/109596-livox-mid-360-lidar-sensor-user-guide). SDK: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2.git) (C++). Pi 5 + ROS2 works; **no official plain-Python serial driver**. US street ~$600–900.

**Benewake / RoboSense / Hesai / Ouster:** no cheap 360° 2D. Benewake TF* is 1D. Hesai/Ouster start in the thousands. Skip.

---

## Recommendation

Buy the **RPLIDAR S2L**. This robot sits in Texas sun and rain; 30–60 klux open-frame DTOFs (LD19, C1, STL-19P, T-mini) will go blind at noon and take water through the window. S2L is sealed IP65, 80 klux DTOF, 5 cm min, 0.12°, 32 kHz, 5 V UART, and has a real Python wrapper (`pyrplidarsdk`) on the official SDK. ±30 mm is coarser than STL-19P’s ±10 mm at a post, but a center-mounted T-bar sees gate posts at ~0.4 m, and 32 kHz lets you average. If money is tight, the **D500/STL-19P at $79** is the right close-range sensor — put it on rubber isolators, add a rain hat (not a sealed tube), and bring it in; do not leave it out. If you want 100 klux **and** IP65 in 2D, TG15 at $317; if you want IP67 3D and will swallow Ethernet + C++/ROS, Mid-360.

**Mounting the S2L:** clamp the 77 mm puck on the T-bar with the optical belt (~18 mm window) at post height, cable gland down, silicone isolators between puck and tube (no published G rating — skid-steer vibration is the enemy of every spinner). Do not wrap it in a polycarbonate cylinder; it is already sealed. Keep the window clean; dust and water beads on the cover are the remaining failure mode. Power from a dedicated 5 V 1 A rail, not the Pi’s USB. USB-UART at 1 Mbps (CP2102 / FT232), then `pyrplidarsdk` — skip ROS.

**If you instead buy STL-19P:** a 360° enclosure is a science project. Optical-grade **polycarbonate** (not acrylic — UV yellows) ~2–3 mm, ~88–92% @ 905 nm, expect 10–20% range loss and refraction error if the wall is curved ([PC window notes](https://www.fivestarfabricating.com/blogs/sensor-safe-polycarbonate-what-robotics-engineers-need-to-know-about-lidar-windows); rain droplets on PC covers drop visibility to ~87–99% depending on coating, [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC11124791/)). A sealed box **will condense** (motor heat + Texas day/night). Use a Gore vent + desiccant, or skip the box and use an open-bottom rain hat. Honest version: spend the extra $220 and get the S2L.

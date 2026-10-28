# Safety and regulations

*This is general information, not legal advice. Rules change, so check with the authority before flying.*

## Aviation rules (EU / Romania)
* Drone operations in the EU fall under **Implementing Regulation (EU) 2019/947** and **Delegated
  Regulation (EU) 2019/945**. In Romania the competent authority is the **Romanian Civil Aeronautical
  Authority (AACR)**. National provisions are in the Air Code (Law 21/2020).
* The aircraft weighs ~1.2 kg and has no class marking, so it is a **privately built UAS**. In the *open*
  category it may fly only in subcategory **A3**: far from people and at least 150 m from residential,
  commercial, industrial or recreational areas, below **120 m**, and in **visual line of sight (VLOS)**.
* The **operator** must register with AACR (drones > 250 g) and display the operator ID on the
  aircraft. The **remote pilot** needs the A1/A3 online training and test.
* Controlling the drone "from anywhere in the world over the internet" (the thesis concept) is a
  **beyond-visual-line-of-sight (BVLOS)** operation. That is the **specific** category, which requires an AACR
  operational authorisation based on a risk assessment (SORA) or a standard scenario.
* Check the geographical zones (restricted/prohibited areas, airports, military sites) before every flight.
  Emergency sites are often under temporary flight restrictions, so coordinate with the incident commander.

## Radio
The aircraft uses the Raspberry Pi's Wi-Fi or a 4G modem only (licence-free / operator networks).
Do not add transmitters without checking ANCOM rules.

## Hazards specific to this project
* **High voltage:** the Geiger board generates ~400 V. Capacitors stay charged after power-off.
* **LiPo batteries:** fire risk. Charge in a LiPo bag, never unattended, and store at 3.8 V/cell.
* **Propellers:** remove them for all bench work.
* **Radioactive check sources:** use only exempt sources (e.g. a lantern mantle or a small calibration
  source) and follow their handling instructions. Do not open sealed sources.
* **Measurement limits:** the MQ-2 and the Geiger counter here are **indicative, uncalibrated** instruments.
  Do not use their readings for life-safety decisions. Use certified instruments and qualified personnel.

## Data protection
The aircraft records gas, radiation, attitude and battery data, with **no camera and no personal data**.
If you add a camera, GDPR applies to images of people.

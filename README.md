# idp_m211
codebase for cambridge engineering part 1b integrated design project

build an automated guided vehicle (agv) to deliver packages between locations

tasks
- flashing red/amber light when in operation
- steady red light when package has magnetic field north pole
- steady green light when package has magnetic field south pole
- north pole packages are delivered to left storage unit
- south pole packages are delivered to right storage unit
- * agv going to top deck stops at the stat finish and waits for ir signal indicating green light

constraints
- four tables have differing surface conditions + navigation lines
- packages cannot be raised more than 20mm during delivering
- packages cannot be rotated

guidelines
- most packages delivered within 5 minutes
- penalties for damage
- minimal parts
- documentation (after final comp)

game design
- 4 packages are placed at the start (lower A/B, upper A/B)
- if all are collected, additional ones are placed randomly
- if the agv touches anything but the box, instant reset

scoring
- identifies N/S and delivers to correct depot +15 (stacks)
- delivers to wrong depot +5 (stacks)
- correct led functionality +10
- returns to start box after delivering 2+ boxes +15
- collision or riding pavement -5 + restart (stacks)
- waiting for green light signal +15 (stacks)
- vague health and safety points

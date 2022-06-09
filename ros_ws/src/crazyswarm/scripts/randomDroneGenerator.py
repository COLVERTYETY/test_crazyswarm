import numpy as np
import sys
text = "crazyflies:"

val = 10

if len( sys.argv ) > 1:
    val = int(sys.argv[1])

print(f"val is {val}")

for i in range(val):
    coord = (np.random.rand(3)*4)
    coord = coord.round(2)
    text+= f"\n- channel: 100 \n  id: {i}\n  initialPosition: [{coord[0]}, {coord[1]}, {coord[2]}] \n  type: medium "

print(text)

#save the text to the file
with open("../launch/crazyflies.yaml","w") as f:
    f.writelines(text)


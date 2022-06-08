import numpy as np
text = "crazyflies:"

for i in range(10):
    coord = np.random.rand(3)*10
    coord[2]=0.0
    coord = coord.round(2)
    text+= f"\n- channel: 100 \n  id: {i}\n  initialPosition: [{coord[0]}, {coord[1]}, {coord[2]}] \n  type: medium "

print(text)

#save the text to the file
with open("../launch/crazyflies.yaml","w") as f:
    f.writelines(text)


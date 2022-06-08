import numpy as np
text = "crazyflies:"

for i in range(10):
    coord = np.random.rand(3)*10
    coord[2]=0.0
    text+= f"\n\n- id: {i} \nchannel: 100 \ninitialPosition: {coord.round(2)} \ntype: medium "

print(text)

#save the text to the file
with open("../launch/allCrazyflies.yaml","w") as f:
    f.writelines(text)


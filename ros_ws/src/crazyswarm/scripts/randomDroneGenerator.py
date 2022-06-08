import numpy as np
text = "crazyflies:"

for i in range(10):
    coord = np.random.rand(3)*10
    coord[2]=0.0
    text+= f"\n\n\t- id: {i} \n\tchannel: 100 \n\tinitialPosition: {coord.round(2)} \n\ttype: medium "

print(text)

#save the text to the file
with open("../launch/allCrazyflies.yaml","a") as f:
    f.writelines(text)


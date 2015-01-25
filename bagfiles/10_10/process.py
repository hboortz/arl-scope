import numpy as np

f_in = open("h_0.txt",'r')

def get_values(word):
    flag = True
    done = False
    words = [0,0]
    while flag:
        line = f_in.readline()
        words = line.strip().split()
        if line == "":
            flag = False
            done = True
            words = [0,0]
        elif word in words:
            flag = False
    return words[1],done

data = {0:[],1:[],2:[],5:[]}
done = False
while not done:
    fid,done = get_values("id:")
    if done:
        break
    else:
        x,done = get_values("x:")
        y,done = get_values("y:")
        z,done = get_values("z:")
        data[float(fid)].append((float(x),float(y),float(z)))

center = []
size = len(data[0])

a_data = zip(*data[0])
b_data = zip(*data[1])
c_data = zip(*data[2])
f_data = zip(*data[5])

mark_locations = []
for mark in [a_data,b_data,c_data,f_data]:
    x = np.mean(mark[0])*39.3701
    y = np.mean(mark[1])*39.3701
    z = np.mean(mark[2])*39.3701
    print x,y,z
    mark_locations.append( (x,y,z) )
average = np.mean(mark_locations,axis=0)
print average[0]
print average[1]
print average[2]


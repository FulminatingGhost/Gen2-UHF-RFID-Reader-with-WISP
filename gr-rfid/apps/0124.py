import matplotlib.pyplot as plt
import sys
import csv
import math
from sklearn.model_selection import KFold
from sklearn.neural_network import MLPClassifier

def hex2bin(hex_str):
    match = {"0": [0, 0, 0, 0], "1": [0, 0, 0, 1],  "2": [0, 0, 1, 0], "3": [0, 0, 1, 1], "4": [0, 1, 0, 0],
    "5": [0, 1, 0, 1], "6": [0, 1, 1, 0],  "7": [0, 1, 1, 1], "8": [1, 0, 0, 0], "9": [1, 0, 0, 1],
    "A": [1, 0, 1, 0], "B": [1, 0, 1, 1],  "C": [1, 1, 0, 0], "D": [1, 1, 0, 1], "E": [1, 1, 1, 0], "F": [1, 1, 1, 1]
    }
    ret = []
    for l in hex_str:
        ret += match[l]
    return ret

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def bin2hexstr(bin_data):
    ret = [str(i) for i in bin_data]
    bstr = "".join(ret)
    hstr = "%0*X" % ((len(bstr) + 3) // 4, int(bstr, 2))
    return hstr

def dec2binstr(dec_data):
    n = int(dec_data) 
    digits = "01"
    x = n % 2
    rest = n / 2
    if rest == 0:
       return digits[x]
    return dec2binstr(rest) + digits[x]

class Signal:
    values = []   # sample values
    fn = ""       # from filename
    epc = []      # epc data
    decoded = []  # decoded data
    pre_idx = 0   # preamble index
    
    def __init__(self, fn, in_data):
        self.values = [float(s) for s in in_data[1:]]
        self.fn = fn
        #self.epc = hex2bin(in_data[0])
        self.epc = list(dec2binstr(in_data[0]).zfill(16))
        self.epc = [int(s) for s in self.epc]

class DecodingScheme:
    name = ""
    algo = None
    results = {}

    def __init__(self, name, algo):
        self.name = name
        self.algo = algo
        self.results = {}

    def execute(self, test_set):
        print("================= %s ================" % self.name)
        for fn in sorted(test_set):
            suc = 0
            fail = 0
            if not fn in self.results:
                self.results[fn] = [0, 0]    # ( # of suc, # of fail)
            for rd in test_set[fn]:
                rd.decoded = self.algo(rd.values[rd.pre_idx:])
                #print(bin2hexstr(rd.decoded))
                #print(bin2hexstr(rd.epc ))
                #print()
                if rd.decoded[2:] == rd.epc[2:]:
                    suc += 1
                else:
                    fail += 1
            print("%s %d %d %.2f" % (fn, suc, fail, float(suc) / (suc+fail)))
            self.results[fn][0] += suc
            self.results[fn][1] += fail


def read_file(filename):
    Signals = {}
    for fn in filename:
        num_data = 0
        tmp = []
        with open(fn, "r") as csvfile:
            reader = csv.reader(csvfile, delimiter=",")
            for s in reader:
                tmp.append(Signal(fn, s))
                num_data += 1
                if num_data >= 200:
                    break
        Signals[fn] = tmp
        print(fn, "read complete", len(Signals[fn]))
    return Signals

def detect_preamble_cc(in_data):
    avg = sum(in_data) / len(in_data)
    avg_data = [ s-avg for s in in_data ] 
    norm = 0
    for d in avg_data:
        norm += d*d
    norm = math.sqrt(norm)
    if isclose(norm, 0.0):
        return -1
    
    #preamble mask
    mask = [1.0]*14       # 1
    mask += [-1.0]*7      # 2
    mask += [1.0]*7
    mask += [-1.0]*14     # 3
    mask += [1.0]*7       # 4
    mask += [-1.0]*7
    mask += [-1.0]*14     # 5
    mask += [1.0]*14      # 6

    cc_value = [0 for i in range(50)]
    max_idx = 0
    max_score = 0
    T1 = range(15, 44)
    for i in T1:
        score = 0.0 # correlation score
        for j in range(len(mask)):
            score += avg_data[i+j] * mask[j]
        score = score / norm
        cc_value[i] = score
        if max_score < score:
            max_idx = i
            max_score = score
    #print("[preamble detected] idx(%d) score(%f)" % (max_idx, max_score))
    return max_idx


def detect_data_cc(in_data):
    preamble = in_data[:LEN_PREAMBLE]
    avg_amp = sum(preamble) / LEN_PREAMBLE
    in_data = [ s-avg_amp for s in in_data[int(LEN_PREAMBLE-LEN_BIT*0.5):] ] + [ 0 for i in range(20)]
    #plt.plot(in_data)
    #plt.show()
    ret = []
    # FM0 mask
    #mask0a = (-1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1 )
    #mask0b = (1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1 )
    #mask1a = (1, 1, 1, 1, 1, 1, 1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1)
    #mask1b = (-1, -1, -1, -1, -1, -1, -1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1)

    mask0a = [ -1 for s in range(25) ] + [ 1 for s in range(25) ] + [ -1 for s in range(25) ] + [ 1 for s in range(25) ]
    mask0b = [ 1 for s in range(25) ] + [ -1 for s in range(25) ] + [ 1 for s in range(25) ] + [ -1 for s in range(25) ]
    mask1a = [ 1 for s in range(25) ] + [ -1 for s in range(50) ] + [ 1 for s in range(25) ]
    mask1b = [ -1 for s in range(25) ] + [ 1 for s in range(50) ] + [ -1 for s in range(25) ]
    mask0a = tuple(mask0a)
    mask0b = tuple(mask0b)
    mask1a = tuple(mask1a)
    mask1b = tuple(mask1b)

    data = { mask0a: 0, mask0b: 0, mask1a: 1, mask1b: 1 }
    state = 1

    for nbits in range(16):
        if state == 1:
            scores = { mask0b: 0, mask1a: 0}
        else:
            scores = { mask0a: 0, mask1b: 0}
        i = int(LEN_BIT*nbits)
        j = i+100
        chunk = in_data[i:j]
        
        max_score = -987654321
        for mask in scores:
            for i, sample in enumerate(mask):
                if i < len(chunk):
                    scores[mask] += chunk[i] * float(sample)
            if max_score < scores[mask]:
                max_mask = mask
                max_score = scores[mask]
        #plt.plot(chunk)
        #print(scores)
        #plt.show()
        # FM0 state transition
        if state == 1 and data[max_mask] == 1:
            state = 0
        elif state == 0 and data[max_mask] == 1:
            state = 1
        ret.append(data[max_mask])

    return ret

def detect_data_cc_with50(in_data):
    preamble = in_data[:LEN_PREAMBLE]
    avg_amp = sum(preamble) / LEN_PREAMBLE
    in_data = [ s-avg_amp for s in in_data[int(LEN_PREAMBLE):] ] + [ 0 for i in range(20)]
    #plt.plot(in_data)
    #plt.show()
    ret = []
    # FM0 mask
    #mask0a = (-1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1 )
    #mask0b = (1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1 )
    #mask1a = (1, 1, 1, 1, 1, 1, 1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1)
    #mask1b = (-1, -1, -1, -1, -1, -1, -1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1)

    mask0a = [ -1 for s in range(12) ] + [ 1 for s in range(13) ] + [ -1 for s in range(13) ] + [ 1 for s in range(12) ]
    mask0b = [ 1 for s in range(12) ] + [ -1 for s in range(13) ] + [ 1 for s in range(13) ] + [ -1 for s in range(12) ]
    mask1a = [ 1 for s in range(12) ] + [ -1 for s in range(25) ] + [ 1 for s in range(13) ]
    mask1b = [ -1 for s in range(12) ] + [ 1 for s in range(25) ] + [ -1 for s in range(13) ]
    mask0a = tuple(mask0a)
    mask0b = tuple(mask0b)
    mask1a = tuple(mask1a)
    mask1b = tuple(mask1b)

    data = { mask0a: 0, mask0b: 0, mask1a: 1, mask1b: 1 }
    state = 1

    for nbits in range(16):
        if state == 1:
            scores = { mask0b: 0, mask1a: 0}
        else:
            scores = { mask0a: 0, mask1b: 0}
        i = int(LEN_BIT*nbits)
        j = i+50
        chunk = in_data[i:j]
        
        max_score = -987654321
        for mask in scores:
            for i, sample in enumerate(mask):
                if i < len(chunk):
                    scores[mask] += chunk[i] * float(sample)
            if max_score < scores[mask]:
                max_mask = mask
                max_score = scores[mask]
        #plt.plot(chunk)
        #print(scores)
        #plt.show()
        # FM0 state transition
        if state == 1 and data[max_mask] == 1:
            state = 0
        elif state == 0 and data[max_mask] == 1:
            state = 1
        ret.append(data[max_mask])

    return ret



def detect_data_nn(in_data):
    # FM0 state
    type1b = [0.0, 0.0, 1.0, 0.0]
    type0a = [1.0, 0.0, 0.0, 0.0]
    type0b = [0.0, 1.0, 0.0, 0.0]
    type1a = [0.0, 0.0, 0.0, 1.0]
    state = type1a

    in_data = in_data[int(LEN_PREAMBLE-LEN_BIT*0.5):]
    in_data += [ 0 for s in range(20) ]
    #peak = max(in_data)
    #in_data = [s/peak for s in in_data]
    peak = max(in_data)
    mi = min(in_data)
    #in_data = [(s-mi)/(peak-mi) for s in in_data]
    in_data = [s/peak for s in in_data]

    ret = []
    for nbits in range(16):
        i = int(LEN_BIT*nbits)
        j = i + 100
        chunk = in_data[i:j]
        #avg = sum(in_data[max(0, i-400):j]) / (j-max(0, i-400))
        #chunk = [(s)/avg if s < 3*avg else 3.0 for s in chunk]
        output = clff_nn.predict([chunk + state])[0]
        if isclose(output[0], 1.0) or isclose(output[1], 1.0):
            ret.append(0)
        else:
            ret.append(1)
        # FM0 state transition
        if state == type1b or state == type0b:
            if ret[nbits] == 0:
                state = type0b
            else:
                state = type1a
        elif state == type0a or state == type1a:
            if ret[nbits] == 0:
                state = type0a
            else:
                state = type1b
    return ret

def getModel(train_set):
    in1 = []
    out1 = []
    for rd in train_set:
        # FM0 state
        type1b = [0.0, 0.0, 1.0, 0.0]
        type0a = [1.0, 0.0, 0.0, 0.0]
        type0b = [0.0, 1.0, 0.0, 0.0]
        type1a = [0.0, 0.0, 0.0, 1.0]
        state = type1a

        pre_idx = 29
        avg_amp = sum(rd.values[pre_idx:pre_idx+LEN_PREAMBLE]) / LEN_PREAMBLE
        in_data = rd.values[int(pre_idx+LEN_PREAMBLE-LEN_BIT*0.5):]
        in_data += [0 for s in range(20) ]
        peak = max(in_data)
        mi = min(in_data)
        #in_data = [(s-mi)/(peak-mi) for s in in_data]
        in_data = [s/peak for s in in_data]

        for nbits in range(2, 16):
            i = int(LEN_BIT*nbits)
            j = i + 100
            chunk = in_data[i:j]
            #avg = sum(in_data[max(0, i-400):j]) / (j-max(0, i-400))
            #chunk = [(s)/avg if s < 3*avg else 3.0 for s in chunk]
            # input format: 28 samples + state
            in1.append(chunk + state)
            # FM0 state transition
            if state == type1b or state == type0b:
                if rd.epc[nbits] == 0:
                    state = type0b
                else:
                    state = type1a
            elif state == type0a or state == type1a:
                if rd.epc[nbits] == 0:
                    state = type0a
                else:
                    state = type1b
            
            out1.append(state)     
    clff_nn = MLPClassifier(verbose=False, max_iter=1000, hidden_layer_sizes=(100, 50, 25))
    clff_nn.fit(in1, out1)
    return clff_nn

LEN_BIT = 49.2
LEN_PREAMBLE = int(LEN_BIT*6)

if __name__ == "__main__":
    #filename = ["50c","50l","50r","80l","80c","80r","110l","110c","110r","140l","140c","140r","170l","170c","170r"] 
    filename = ["100_0", "200_0", "300_0", "400_0", "100_50l_0", "100_100l_0", "200_50l_0", "200_100l_0", "300_50l_0", "300_100l_0", "400_50l_0", "400_100l_0","100_50r_0", "100_100r_0", "200_50r_0", "200_100r_0", "300_50r_0", "300_100r_0", "400_50r_0", "400_100r_0", "100_0_45" , "100_0_90", "100_0_135", "200_0_45" , "200_0_90", "200_0_135", "300_0_45" , "300_0_90", "300_0_135", "400_0_45" , "400_0_90", "400_0_135" ]
    #filename += ["50_0", "50_45", "50_90", "50_135", "50_180", "50_225", "50_270", "50_315", 
    #    "100_0", "100_45", "100_90", "100_135", "100_180", "100_225", "100_270", "100_315",  
    #    "150_0", "150_45", "150_90", "150_135", "150_180", "150_225", "150_270", "150_315"] 
    #filename = [ sys.argv[1] ]
    filename = [ "50mw/" + fn + "data.csv" for fn in filename ]

    data_set = read_file(filename)

    num_fold = 5
    train_data = [ [] for i in range(num_fold) ]
    test_data = [ {} for i in range(num_fold) ]

    for fn in sorted(data_set):
        for rd in data_set[fn]:
            #rd.pre_idx = detect_preamble_cc(rd.values)
            rd.pre_idx = 29

        kf = KFold(n_splits=num_fold)
        fold = 0
        for i in range(num_fold):
            test_data[i][fn] = []
        for train_index, test_index in kf.split(data_set[fn]):
            #if not (("45" in fn) or ("90" in fn) or ("135" in fn)):
            train_data[fold] += [data_set[fn][i] for i in train_index ]
            test_data[fold][fn] += [ data_set[fn][i] for i in test_index ]
            fold += 1

    # based cross correlation + state
    cc = DecodingScheme("cc", detect_data_cc)
    cc_50 = DecodingScheme("cc_50", detect_data_cc_with50)
    # based neural network
    nn = DecodingScheme("nn", detect_data_nn)

    for i in range(num_fold):
        cc.execute(test_data[i])
        cc_50.execute(test_data[i])
        #clff_nn = getModel(train_data[i])
        #nn.execute(test_data[i])
    #print("================= %s ================" % cc.name)
    for fn in sorted(cc.results):
        #print(fn, cc_50.results[fn], nn.results[fn])
        print(fn, cc.results[fn], cc_50.results[fn])
    """
    fp = open("one.out", "a")
    fp.write(fn + "\t")
    #fp.write(str(cc.results[fn]) + "\t")
    fp.write(str(nn.results[fn]) + "\t")
    fp.write("\n")
    fp.close()
    """
    #print("================= %s ================" % cc_50.name)
    #for fn in sorted(cc_50.results):
    #    print(fn, cc_50.results[fn])
    #print("================= %s ================" % nn.name)
    #for fn in sorted(nn.results):
    #    print(fn, nn.results[fn])


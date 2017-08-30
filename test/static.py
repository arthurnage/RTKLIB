#!/usr/bin/python3

from functools import reduce
import csv
import subprocess
import numpy
import re
import os
import math
import time

from scipy.optimize import fmin

start = time.time()

# returns a list of parameters like "./rnx2rtkp rov.obs base.obs rov.nav etc..."
def get_params(programs, configs):
    params = []
    data = list((x + '/', z) for x, y, z in os.walk("test"))
    # looking for tests in path RTKLIB/test/test/"test_type"/"test_num" (digit)
    data = list(filter(lambda x: re.search('/static/\d', x[0]), data))  # looking for only static logs
    data = list(map(lambda x: [os.path.abspath('.') + '/' + x[0], x[1]], sorted(data)))
    for a in data:
        print(a[0])
    program = './../app/rnx2rtkp/gcc/rnx2rtkp '
    for x in data:
        rover_obs = x[0] + 'rover.obs '
        rover_nav = x[0] + 'rover.nav '
        base_obs = x[0] + 'base.obs '
        # config = '-k ' + x[0] + 'rtk.conf '
        outdata = '-l 59.6625316333 30.8075323222 52.034 -o ' + 'test/outdata/static/test_' + str(data.index(x))
        # hardcoded base presice coordinates
        # outdata = '-l 59.662531660 30.807532784 52.0110 -o ' + 'test/outdata/static/test_' + str(data.index(x))
        # outdata = '-o ' + 'test/outdata/static/test_' + str(data.index(x))
        # we know for sure that all files below exist
        if x[1].count('rover.obs'):
            if x[1].count('rover.nav'):
                if x[1].count('base.obs'):
                    params.append(rover_obs + base_obs + rover_nav + outdata)
                else:
                    params.append(rover_obs + rover_nav + outdata)
    params = [prog + ' ' + param + '_prog_' + str(programs.index(prog)) + '_config_' + str(configs.index(config)) +
              '.pos -k ' + config for prog in programs for config in configs for param in params]
    return params


# returns stderr output data from rnx2rtkp
def get_data(process):
    x = str(process.communicate()[1]).split('\\r')
    return [int(f[-1:]) for f in x if re.search(r'Q=\d$', f)]

def get_mean_value(i, data):
    return round(numpy.mean(list(map(lambda x: 100 * x.count(i) / len(x), data))), 3)

def parse_pos(name):
    file = open(name, 'r')
    temp = list(map(lambda x: x.split()[2:6], list(filter(lambda x: x[0] == "2", file.readlines()))))
    return list(map(lambda x: list(map(lambda y: float(y), x)), temp))

def deviation_distance(x, tests, etalon):
    sum = 0.0
    res = []
    for j in range(len(tests)):
        i = 0
        while i < len(tests[j]):
            sum += math.sqrt((x[0] + etalon[0] - tests[j][i][0]) ** 2 + (x[1] + etalon[1] - tests[j][i][1]) ** 2 +
                             (x[2] + etalon[2] - tests[j][i][2]) ** 2)
            i += 100
        res.append(sum)
    return list(map(lambda x: x * 100 / len(etalon), res))

def compare_rows(first, second):
    first = first.split(';')
    second = second.split(';')
    out = []
    for i in range(len(first)):
        if first[i].replace('.', '', 1).isdigit():
            if i == 1 or i == 2 or i == 7:
                num = int(float(second[i]) - float(first[i]))
            else:
                num = float(second[i]) - float(first[i])
            try:
                if i >= (len(first) - 3):
                    num = num * 100 / float(first[i])
                if num >= 0 and i > 1:
                    num = '+' + str(round(num, 3))
                else:
                    num = str(round(num, 3))
            except ZeroDivisionError:
                print("empty data\n")
            out.append(num)
        else:
            word = first[i]

            if word[-1] == 'm':
                word = word[:-2]

            if i >= (len(first) - 3):
                word += ' %'
            out.append(word)
    return out

def make_comparison_scv(first, second):
    first_csv = open(first, 'r')
    second_csv = open(second, 'r')
    compare_csv = open('comparison.csv', 'w')
    first_reader = list(csv.reader(first_csv))
    second_reader = list(csv.reader(second_csv))
    compare_writer = csv.writer(compare_csv, delimiter=';')
    for i in range(len(first_reader)):
        compare_writer.writerow(compare_rows(first_reader[i][0], second_reader[i][0]))

sol_status = {
    0: "No solution",
    1: "Fixed",
    2: "Float",
    3: "SBAS",
    4: "DGPS",
    5: "Single"
}

programs = [a[:-1] for a in open('test/exec.txt').readlines()]
configs = [a[:-1] for a in open('test/config.txt').readlines()]

# parallel start of rnx2rtkp
params = get_params(programs, configs)
FNULL = open(os.devnull, 'w')
processes = [subprocess.Popen(params[i].split(), stderr=FNULL) for i in range(len(params))]
exit_codes = [p.wait() for p in processes]

print("\ntime_: {}".format(time.time() - start))

tests = [parse_pos('test/outdata/static/test_{0}_prog_{1}_config_{2}.pos'.format(i, 0, j)) for j in
         range(2) for i in range(len(params) // 2)]

out_data = [[int(a[3]) for a in tests[i]] for i in range(len(tests))]
out_data = [out_data[:len(out_data) // 2], out_data[len(out_data) // 2:]]
tests = [[a[:3] for a in tests[i]] for i in range(len(tests))]

tests = [tests[:len(tests) // 2], tests[len(tests) // 2:]]
etalon = list(map(lambda y: y.split(), open('test/etalon_xyz.txt', 'r').readlines()))
etalon = list(map(lambda x: list(map(lambda y: float(y), x)), etalon))

sigma = [[[math.sqrt(sum([(etalon[k][i] - tests[l][k][j][i]) ** 2 for i in range(3)]))
           for j in range(len(tests[l][k]))] for k in range(len(tests[l]))] for l in range(len(tests))]

print("all")
for j in range(len(sigma)):
    # print("rnx2rtkp ver. {}".format(j))
    print("config ver. {}\n".format(j))
    for i in range(len(sigma[j])):
        print("test number {}: ".format(i))
        print("max deviation: {}".format(round(max(sigma[j][i]), 3)))
        print("mean value: {}".format(round(numpy.mean(sigma[j][i]), 3)))
        print("standart deviation: {}\n".format(round(numpy.std(sigma[j][i]), 3)))

all_tests_data = [[
    "all",
    sum(list(map(lambda x: len(x), out_data[j]))),
    reduce(lambda f, x: f + x.count(0), out_data[j], 0),
    get_mean_value(1, out_data[j]),
    get_mean_value(2, out_data[j]),
    get_mean_value(3, out_data[j]),
    get_mean_value(4, out_data[j]),
    reduce(lambda f, x: f + x.count(5), out_data[j], 0),
    round(numpy.mean([max(s) for s in sigma[j]]), 3),
    round(numpy.mean([numpy.mean(s) for s in sigma[j]]), 3),
    round(numpy.mean([numpy.std(s) for s in sigma[j]]), 3)
] for j in range(len(sigma))]

for l in range(len(sigma)):
    # csv_file = open('rnx2rtkp_version_{}.csv'.format(l), 'w')
    csv_file = open('config_version_{}.csv'.format(l), 'w')
    writer = csv.writer(csv_file, delimiter=';')
    writer.writerow(["Test number", "Point's number", "No solution", "Fixed %", "Float %",
                    "SBAS %", "DGPS %", "Single", "Max deviation m", "Mean deviation m", "std m"])
    writer.writerow(all_tests_data[l])
    for j in range(len(out_data[l])):
        row = ["Test num. {0}".format(j), len(out_data[l][j])]
        for i in range(0, 9):
            try:
                if i == 0 or i == 5:
                    row.append(out_data[l][j].count(i))
                elif i == 6:
                    row.append(round(max(sigma[l][j]), 3))
                elif i == 7:
                    row.append(round(numpy.mean(sigma[l][j]), 3))
                elif i == 8:
                    row.append(round(numpy.std(sigma[l][j]), 3))
                else:
                    row.append(round(100 * out_data[l][j].count(i) / len(out_data[l][j]), 3))
            except ZeroDivisionError:
                print("empty data\n")
        writer.writerow(row)
    csv_file.close()

for j in range(len(sigma)):
    csv_file = open('config_version_{}.csv'.format(l), 'r')
    reader = csv.reader(csv_file)
    # for row in reader:
    # print(" ".join(row))

make_comparison_scv('config_version_0.csv', 'config_version_1.csv')

end = time.time()
print("\ntime: {}".format(end - start))


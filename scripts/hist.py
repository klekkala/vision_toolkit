import matplotlib.pyplot as plt
import pickle
# Example lists

with open('length.pkl', 'rb') as f:  # 'rb' stands for read binary mode
    list1 = pickle.load(f)

with open('length_test.pkl', 'rb') as f:  # 'rb' stands for read binary mode
    list2 = pickle.load(f)

with open('length_new.pkl', 'rb') as f:  # 'rb' stands for read binary mode
    list3 = pickle.load(f)

# list1_sec = []
# for duration in list1:
#     duration_in_seconds = duration.secs + duration.nsecs / 1e9
#     list1_sec.append(float(duration_in_seconds))

# list2_sec = []
# for duration in list2:
#     duration_in_seconds = duration.secs + duration.nsecs / 1e9
#     list2_sec.append(float(duration_in_seconds))


# list3_sec = []
# for duration in list3:
#     duration_in_seconds = duration.secs + duration.nsecs / 1e9
#     list3_sec.append(float(duration_in_seconds))

# list1,list2,list3=list1_sec,list2_sec,list3_sec

# Plotting histograms
plt.hist([list1+list3, list1+list2], alpha=0.5, color=['orange', 'blue'], label=['After splitting', 'Original'])
plt.xlabel('Episode Length')
plt.ylabel('Frequency')
plt.legend()
plt.grid(True)
plt.savefig('h123.png')

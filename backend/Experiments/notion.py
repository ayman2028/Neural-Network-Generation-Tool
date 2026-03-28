
from collections import defaultdict

class document:
    def __init__(self, name, text, time=0):
        self.name = name
        self.time = time
        self.timestamps = defaultdict(dict) #Using a map because I am assuming
        # that inserts happen more than retrievals. O(1) insert.
        self.timestamps[time] = {"name" : name, "text":text}

    def createTimeStamp(self, text, time): #O(1)
        self.timestamps[time]["text"] = text

    def getLatestTime(self): #O(N)
        #times = sorted(list(self.timestamps.keys()))
        #return times[-1]
        mx = 0
        for t in self.timestamps.keys():
            mx = max(mx, t)
        return mx
            

    def save(self, text, time): #O(1)
        self.timestamps[time] = {"name" : self.name, "text" : text}

    def getTextAtTime(self, time): #O(NlogN)
        #Come back and do binary search.
        if time > self.getLatestTime() or time <0 : return ""
        if time in self.timestamps:
            return self.timestamps[time]["text"]
        times = sorted(list(self.timestamps.keys()), reverse = True)
        findT = time
        index  = 0
        while index < len(times) and findT < times[index]:
            index += 1
        return self.timestamps[times[index]]["text"]

    def getLatestDoc(self): #O(N)
        time = self.getLatestTime()
        return self.timestamps[time]["text"]


class DocumentStore:
    def __init__(self):
        self.library = {} # Key - name, value - document object
    def save(self, name, text): #Time complexity is O(1)
        if name not in self.library:
            doc = document(name, text)
            self.library[name] = doc
        else:
            """If it is in there we will want to overwrite it instead
            While still preserving older versions."""
            t = self.library[name].getLatestTime()
            self.library[name].save(text, t+1)
            
    def saveAtTime(self, name, text, timestamp): #Time complexity O(1)
        if name not in self.library:
            doc = document(name, text, timestamp)
            self.library[name] = doc
        else:            
            self.library[name].save(text, timestamp)


    ''' Return the current contents of the file w/ this file name. If no file exits, return
    nothing.'''
    def get(self, name): #Time complexity O(N)
            if name in self.library:
                return self.library[name].getLatestDoc()
            else:
                return None

    ''' Return the contents of the file at a given timestamp. '''
    def get_at_timestamp(self, name, timestamp): #O(NlogN)
            if name not in self.library:
                return ""
            else:
                return self.library[name].getTextAtTime(timestamp)
            

"""Things to test for:
    Simple fill in with a single document.
    Update document and then get older versions. 
        Check if new version is still there
    Overwrite a document
    edge cases
"""
def simpleSingleTest():
    DS = DocumentStore()
    DS.save("Doc1", "Version1")
    test = DS.get("Doc1")
    assert test == "Version1", "Didn't store document"
    DS.save("Doc2", "D2V1")
    test = DS.get("Doc2")
    assert test == "D2V1", "Couldn't store second document"
    DS.save("Doc1", "TimeStamp1, Test overwrite")
    test = DS.get("Doc1")
    assert test == "TimeStamp1, Test overwrite", "Did not overwrite doc1"
    test = DS.get_at_timestamp("Doc1", 1)
    assert test == "TimeStamp1, Test overwrite", "Timestamp working correctly"
    test = DS.get_at_timestamp("Doc1", 0)
    assert test == "Version1", "Was able to get older version"

    #Going to test the timestamp update.
    DS.saveAtTime("Doc1", "timestamp 10", 10)
    test = DS.get_at_timestamp("Doc1", 10)
    assert test == "timestamp 10", "Did not save a new timestamp"
    test = DS.get_at_timestamp("Doc1", 1)
    assert test == "TimeStamp1, Test overwrite", "Original timestamp no longer there"
    test = DS.get_at_timestamp("Doc1", 5)
    assert test == "TimeStamp1, Test overwrite", f"unable to get previous timestamp, got: {test} "
    test = DS.get_at_timestamp("Doc1", 11)
    assert test == "", f"Able to get previous timestamp, got: {test} "
    print("test1 success")

def main():
    simpleSingleTest()

"""
For optimization, I would keep track of the latest time when doing a write, would bring certain reads down.
I would also add a binary search for getting at a certaintime.
"""
if __name__ == "__main__":
    main()
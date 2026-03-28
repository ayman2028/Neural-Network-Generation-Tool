import re
from collections import defaultdict

class database:
    pageLinks = defaultdict(list)
    #Will manage rows and columns.
    def __init__(self, pageId = 0):
        self.rows = {} #key = name of what to add, value is the value
        self.columnName = set() #Will have the names of the columns in the list. 
        self.pageId = pageId
        self.related = set() #List of all pages that have a DB that have a link to this.
    def insert(self, input : dict):
        assert len(self.rows)< 100, "Too many rows"
        rowid = len(self.rows)
        for k in input.keys():
            if k not in self.columnName:
                    assert len(self.columnName)<1000, "Too many columns"
                    self.columnName.add(k)
        for k,v in input.items(): #Here should be the links to the pages.By pageId
            if re.match(r"^links", k): # Assume links will be followed by a list of links
                for link in v:
                    self.pageLinks[self.pageId].append(link)
                pass  # TODO: implement link handling
        
        self.rows[rowid] = input

    def sort(self, keys: list[str], order: list[bool]):
        assert len(keys) == len(order), "not enough parameters for sort"
        vals = list(self.rows.values())
        def key_fn(key):
            #We are sorting for key, we also get each argument for vals.
            def _f(item):
                #Item is the argument from vals. 
                #we see if it is missing, then if false it will go in front of the list.
                missing = key in item
                return (missing, item.get(key,None))
            return _f
        for key, asc in reversed(zip(keys,order)):
            vals.sort(key= key_fn(key),reverse = not asc)
        return vals
        

                




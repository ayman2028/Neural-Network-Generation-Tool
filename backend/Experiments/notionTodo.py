from typing import List, Optional
class Task:
    def __init__(self, title):
        self.title = title
        self.status = False
        self.subTasks = []

    def addSubTask(self, title):
        self.subTasks.append(Task(title))
    
    def markComplete(self):
        self.status = True
        for s in self.subTasks:
            s.markComplete()

    def toStr(self):
        s = f"Title:{self.title}"
        lines = []
        lines.append(s)
        s = f"{'complete' if self.status else 'incomplete'}"
        lines.append(s)
        if len(self.subTasks) >0:
            lines.append("Subs:")
        for sub in self.subTasks:
            slines = sub.toStr()
            for line in slines:
                lines.append('\t'+line)
            #add = "\n".join([("\t" + line) for line in slines])
            #ines.append(add)
        return lines
    
    """You have to think about how you are going to find the tasks to mark them complete"""
    def find(self, title)->Optional["Task"]:
        if self.title == title:
            return self
        for sub in self.subTasks:
            hit = sub.find(title)
            if hit:
                return hit
        return None

            

class TodoList:
    def __init__(self):
        self.tasks = []

    def addTask(self, title:str):
        newT = Task(title)
        self.tasks.append(newT)
        print("New Task "+title)
        
    def find(self, title):
        for t in self.tasks:
            if t.find(title):
                return t.find(title)
        return None
    def addSubTask(self, parent, title):
        parentTask = self.find(parent)
        parentTask.addSubTask(title)

    def markComplete(self, title):
        mark = self.find(title)
        assert mark!=None, "No such task"
        mark.markComplete()
    def display(self):
        for task in self.tasks:
            print("\n".join(task.toStr()))

def testCase():
    TList = TodoList()
    
    TList.addTask("Home Work")
    TList.addSubTask("Home Work", "Math")
    TList.addSubTask("Home Work", "Science")
    TList.addSubTask("Science", "Essay")
    TList.addTask("Chores")
    TList.markComplete("Science")
    TList.display()
testCase()
    

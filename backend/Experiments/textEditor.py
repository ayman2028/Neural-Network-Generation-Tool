
class Operation:
    def apply(self, text :str)->str:
        raise Exception("Not implmented.")
    def rollbackOperation(self):
        raise Exception("Not Implemented.")
    

class addToEnd(Operation):
    def __init__(self, chars : str):
        self.chars_to_add = chars
    def apply(self, text : str)->str:
        return text + self.chars_to_add
    def rollbackOperation(self):
        return deleteFromEnd(len(self.chars_to_add))
    
class deleteFromEnd(Operation):
    def __init__(self,NoToDel : int):
        self.NoToDel = NoToDel
        self.charsDeleted = ""
    def apply(self, text:str):
        """This will delete the last indicated chars, store them for rollback, then return modified text.
        EC: What if negative number, what if greater than current len.
        """
        if len(text) <= self.NoToDel:
            self.charsDeleted = text
            return ""
        self.charsDeleted = text[-self.NoToDel:]
        return text[:-self.NoToDel]
    
    def rollbackOperation(self):
        """Create an add Operation with the text that was deleted."""
        return addToEnd(self.charsDeleted)

class replaceAndASnap(Operation):
    def __init__(self, chars: str):
        self.newDoc = chars
        self.oldDoc = ""
    def apply(self, text):
        self.oldDoc = text
        return self.newDoc
    def rollbackOperation(self):
        return replaceAndASnap(self.oldDoc)

class TextEditor:
    def __init__(self):
        self.doc = Document()
        self.undo_limit = 10
        self.undo_stack : list[Operation] = []
        self.redo_stack : list[Operation] = []

    def addUndo(self, Op:Operation):
        self.undo_stack.append(Op)
        if self.undo_limit < len(self.undo_stack):
            self.undo_stack.pop(0)

    def addRedo(self, Op:Operation):
        self.redo_stack.append(Op)
        if self.undo_limit < len(self.redo_stack):
            self.redo_stack.pop(0)

    def apply_operation(self,Op : Operation):
        """
        Takes Op, applys the operation to text, then gets the rollback then pushes that to the undo.
        Then it clears the redo stack.
        EC: Operation results in no changes. Stacks are full.
        """
        rollBack =  self.doc.apply(Op)
        if not rollBack: return None
        self.addUndo(rollBack)
        self.redo_stack.clear()

        
    
    def undo(self):
        """
        Undoes the previous operation. Applys the op, get's rollback and pushes to redo.
        EC: What if undo is empty?
        """
        if not self.undo_stack: return
        
        undoOp = self.undo_stack.pop()
        rollBack = self.doc.apply(undoOp)
        self.addRedo(rollBack)
    
    def redo(self):
        """This pops from redo stack, adds to undo stack."""
        if not self.redo_stack: return
        redoOp = self.redo_stack.pop()
        rollback = self.doc.apply(redoOp)
        self.addUndo(rollback)
    


class Document:
    def __init__(self):
        self.text = ""

    def apply(self, Op)->Operation:
        oldText = self.text
        self.text = Op.apply(self.text)
        if oldText == self.text: return None
        return Op.rollbackOperation()
        
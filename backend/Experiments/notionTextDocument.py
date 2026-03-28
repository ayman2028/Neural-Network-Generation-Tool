
undoLimit = 5

class Operation:
    def apply():
        raise Exception("")
   
    def rollback_operation():
        raise Exception("")
   


class InsertAtEndOperation(Operation):
    def __init__(self, chars_to_insert: str):
        self.chars_to_insert = chars_to_insert
   
    def apply(self, doc: str) -> str:
        return doc + self.chars_to_insert
   
    def rollback_operation(self) -> Operation:
        return DeleteFromEndOperation(len(self.chars_to_insert))

class DeleteFromEndOperation(Operation):
    def __init__(self, num_chars_to_delete: int):
        self.num_chars_to_delete = num_chars_to_delete
        self.deleted_text = ""
   
    def apply(self, doc: str) -> str:
        if len(doc) < self.num_chars_to_delete:
            self.deleted_text = doc
            return ""
        self.deleted_text = doc[-self.num_chars_to_delete:]
        return doc[:-self.num_chars_to_delete]
   
    def rollback_operation(self) -> Operation:
        return InsertAtEndOperation(chars_to_insert=self.deleted_text)
    
class TextDocument:
    def __init__(self, text = ""):
        self.text = text
        self.redo = []
        self.undo = []
    def apply_operation(self, Op : Operation):
        #When you add something: add to undo, clear redo
        #when you delete somethng, add to undo, clear redo
        
        before = self.text
        self.text = Op.apply(self.text)
        if(self.text == before):
            return
        self.redo.clear()
        self.undo.append(Op.rollback_operation())
        if len(self.undo) > undoLimit:
            self.undo.pop(0)
            
    def undo_last(self):
        #operform last operation if ther was one, add it to the redo
        #we don't call apply_operation because it will erase stack
        if len(self.undo) == 0 : 
            return
        op = self.undo.pop()
        self.text = op.apply(self.text)
        self.redo.append(op.rollback_operation())
        if len(self.redo) == undoLimit:
            self.redo.pop(0)

    def redo_last(self):
        #perform redo, push to undo
        if len(self.redo)==0:
            return
        op = self.redo.pop()
        self.text = op.apply(self.text)
        self.undo.append(op.rollback_operation())
        if len(self.undo) == undoLimit:
            self.undo.pop(0)

        
    def get_current_content(self):
        return self.text

def testCase():
    T1 = TextDocument()
    assert T1.get_current_content() == ""
    T1.apply_operation(InsertAtEndOperation("Hello"))
    assert T1.get_current_content() == "Hello"

testCase
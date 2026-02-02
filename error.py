class Except(Exception):
    def __init__(self,message : str,error_code):
        super().__init__()
        self.message = message
        self.error_code = error_code
    
    def __str__(self):
        return f"{self.message}\nfonksiyon: {self.error_code}"

class Fletcher:
    """Specifies the general class for the FletcherNbit checksum"""
    
    modulus = 255
    bitshift = 8
    bytes_read = 1

    def __init__(self):
        self.c0 = 0
        self.c1 = 0
        self.cb0 = 0
        self.cb1 = 0

    def update(self, bytestring):
        """Updates the checksum with the data passed in."""
        
        # prevents unicode or weirdly encoded strings
        # from being passed in naked
#         if type(bytestring) != bytes:
#             bytestring = str.encode(bytestring)
#         bytestring = bytearray(bytestring)

        # an efficient way of doing math.ceil() without frills
        iterations = ((len(bytestring) - 1) // self.bytes_read) + 1

        # actually updating the checksum by iterating through the bytes
        for i in range(0, iterations):
            start = self.bytes_read*i
            end = self.bytes_read*(i+1)
            
            bytepart = int.from_bytes(bytestring[start:end], byteorder="little")
            self.c0 = (self.c0 + bytepart) % self.modulus
            self.c1 = (self.c1 + self.c0) % self.modulus

        # compute check bytes
        self.cb0 = self.modulus - ((self.c0 + self.c1) % self.modulus)
        self.cb1 = self.modulus - ((self.c0 + self.cb0) % self.modulus)


    def hexdigest(self):
        """Parses the checksum and returns it in the form of a hex string"""
        
        translated_number = hex((self.c1 << self.bitshift) + (self.c0))
        if len(translated_number) % 2 == 0:
            return '0x' + translated_number[2:]
        return '0x0' + translated_number[2:]

    def cb_hexdigest(self):
        """Parses the checkbytes and returns it in the form of a hex string"""
        
        translated_number = hex((self.cb0 << self.bitshift) + (self.cb1))
        if len(translated_number) % 2 == 0:
            return '0x' + translated_number[2:]
        return '0x0' + translated_number[2:]
       

class Fletcher16(Fletcher):
    """Contains the specification for the 16-bit checksum"""
    modulus = 255
    bitshift = 8
    bytes_read = 1
    

class Fletcher32(Fletcher):
    """Contains the specification for the 32-bit checksum"""
    modulus = 65535
    bitshift = 16
    bytes_read = 2
    

class Fletcher64(Fletcher):
    """Contains the specification for the 64-bit checksum"""
    modulus = 4294967295
    bitshift = 32
    bytes_read = 4
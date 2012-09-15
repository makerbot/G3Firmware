"""
Portions of this code is a copy of a work-in-progress from Makerbot Inc.'s s3g/ repo,

  https://github.com/makerbot/s3g/blob/master/makerbot_driver/EEPROM/eeprom_analyzer.py

It then has been extended to write some useful Java code for the ReplicatorG source
files JettyEEPROM.java, OnboardParameters.java, and Makerbot4GDriver.java
"""

"""
eeprom_analyzer.py with some slight changes starts here
"""

"""
Analyzes the eeprom map in the Mightyboards Source code's
Eeprom.hh file.  Expects blocks of information in the form of:
//$BEGIN_ENTRY
//$S:<some size>
const static uint16_t <some name> = <some address>

Takes the above information and turns it into a python
dict in the form of:
{
  <some name> : {
      size  : <some size>,
      address : <some address>,
      }
}
"""

class EndOfNamespaceError(IOError):
  def __init__(self):
    pass

class EndOfEepromError(IOError):
  def __init__(self):
    pass

class eeprom_analyzer(object):

  def __init__(self):
    self.file = open('EepromMap.hh', 'r')
    self.eeprom_map = {}
    self.nl = {}
    self.ln = {}
    self.braces = 0

  def parse_file(self):
    self.eeprom_map = {}
    try:
      while True:
        namespace_name = self.find_next_namespace().lower()
        namespace = {}
        try:
          while True:
            self.find_next_entry()
            variables = self.parse_out_variables(self.file.readline())
            (name, location) = self.parse_out_name_and_location(self.file.readline())
            self.nl[name] = location
            self.ln[location] = name
            v = {
                'offset'  : location,
                }
            for variable in variables:
              variable = variable.split(':')
              v[variable[0]] = variable[1]
            namespace[name] = v
        except EndOfNamespaceError:
          self.eeprom_map[namespace_name]  = namespace
    except EndOfEepromError:
        pass

  def find_next_entry(self):
    namespace_end = '}'
    open_brace = '{'
    entry_line = '//$BEGIN_ENTRY'
    line = self.file.readline()
    while entry_line not in line:
      if open_brace in line:
          self.braces += 1
      if namespace_end in line:
          self.braces -= 1
          if self.braces <= 0:
              raise EndOfNamespaceError
      line = self.file.readline()
    return line

  def find_next_namespace(self):
    """
    Reads lines from the file until a line that 
    starts with namespace is encountered.  The name 
    of that namespace is then parsed out and returned.

    @return str name: The name of the current namespace
    """
    namespace = 'namespace'
    end_of_eeprom = '#endif'
    line = self.file.readline()
    while not line.startswith(namespace):
      if end_of_eeprom in line:
        raise EndOfEepromError
      line = self.file.readline()
    self.braces += 1
    return self.parse_out_namespace_name(line)

  def parse_out_namespace_name(self, line):
    """
    Given a line in the form of 
      "namespace <name> {"
    parses out the name
  
    @param str line: The line with the namespace
    @return str name: The name of the namespace 
    """
    line = line.strip()
    for s in ['\n', '\r', '\t', '{']:
      line = line.rstrip(s)
    line = line.lstrip('namespace')
    line = line.replace(' ', '')
    return line

  def parse_out_name_and_location(self, line):
    """
    Given a line in the form of:
      const static uint16_t <name> = <location>;
    parses out the name and location.
    If we get a line not of this form, we will fail.

    @param str line: the line we want information from
    @return tuple: Information in the form of (name, location)
    """
    for w in ['const', 'static', 'uint16_t']:
      line = line.replace(w, '')
    for s in ["\r", "\n", ";"]:
      line = line.rstrip(s)
    line = line.replace('\t', '')
    line = line.replace(" ", "")
    (name, location) = line.split("=")
    return name, location

  def parse_out_variables(self, line):
    line = line.strip()
    for s in ['\n', '\r', '\t',]:
      line = line.rstrip(s)
    line = line.lstrip('//')
    line = line.replace(' ', '')
    parts = line.split('$')
    #Dont return the first, since its empty
    return parts[1:]

  def collate_maps(self, the_map):
#    main_map = 'eeprom_offsets'
#    collated_map = self.eeprom_map[main_map]
    collated_map = the_map
#    for key in self.eeprom_map[main_map]:
    for key in the_map:
#      if 'eeprom_map' in self.eeprom_map[main_map][key]:
      if 'eeprom_map' in the_map[key]:
#        sub_map_name = self.eeprom_map[main_map][key]['eeprom_map']
        sub_map_name = the_map[key]['eeprom_map']
        collated_map[key]['sub_map'] = self.eeprom_map[sub_map_name]
        self.collate_maps(collated_map[key]['sub_map'])
#    collated_map = {'eeprom_map'  : collated_map}
    return collated_map

"""
eeprom_analyzer.py ends here
"""

a = eeprom_analyzer()
a.parse_file()
offsets = a.eeprom_map['eeprom']

print "// EEPROM offsets for JettyEEPROM.java"
print "pacakge replicatorg.drivers.gen3;"
print
print "class JettyEEPROM extends Sanguino3GEEPRPOM {"
for offset in sorted(a.ln):
    n = a.ln[offset]
    print "\tfinal public static int %s = %s;" % (n.ljust(26), offset)
print "}"
print

print "// EEPROMParams for OnboardParameters.java"
print "\tpublic enum EEPROMParams {"
for n in sorted(a.nl):
    o = a.nl[n]
    t = offsets[n]['type']
    if t == 'b':
       tstr = 'int8_t'
    elif t == 'B':
       tstr = 'uint8_t'
    elif t == 'I':
       tstr = 'uint32_t'
    elif t == 'i':
       tstr = 'int32_t'
    elif t == 'q':
       tstr = 'int64_t'
    elif t == 'Q':
       tstr = 'uint64_t'
    if 'exponent' in offsets[n]:
       e = int(offsets[n]['exponent'])
       if e < 0:
          tstr += ' / 10^' + str(-e)
       elif e > 0:
          tstr += ' * 10^' + str(e)
    print "\t\t%s// (%s, %s)" % ((n+',').ljust(28), tstr, o)
print "\t}"
print

# Need a deep copy as we will be deleting members
nl = {}
for k in a.nl:
    nl[k] = a.nl[k]

print "\t@Override"
print "\tpublic int getEEPROMParamInt(EEPROMParams param) {"
print "\t\tswitch (param) {"
for name in sorted(nl):
    d = offsets[name]
    if not 'type' in d:
        continue
    if 'floating_point' in d:
        continue
    t = d['type']
    if t == 'B':
       print "\t\tcase %s : return getUInt8EEPROM(JettyEEPROM.%s);" % (name.ljust(26), name)
       del nl[name]
    elif t == 'b':
       print "\t\tcase %s : return getInt8EEPROM(JettyEEPROM.%s);" % (name.ljust(26), name)
       del nl[name]
    elif t == 'i':
       print "\t\tcase %s : return getInt32EEPROM(JettyEEPROM.%s);" % (name.ljust(26), name)
       del nl[name]
print "\t\tdefault :\n\t\t\tBase.logger.log(Level.DEBUG, \"getEEPROMParamInt(\" + param + \") call failed\");\n\t\t\treturn 0;"
print "\t\t}"
print "\t}"
print

print "\t@Override"
print "\tpublic long getEEPROMParamUInt(EEPROMParams param) {"
print "\t\tswitch (param) {"
for name in sorted(nl):
    d = offsets[name]
    if not 'type' in d:
        continue
    if 'floating_point' in d:
        continue
    t = d['type']
    if t == 'I':
       print "\t\tcase %s : return getUInt32EEPROM(JettyEEPROM.%s);" % (name.ljust(26), name)
       del nl[name]
print "\t\tdefault :\n\t\t\tBase.logger.log(Level.DEBUG, \"getEEPROMParamUInt(\" + param + \") call failed\");\n\t\t\treturn 0L;"
print "\t\t}"
print "\t}"
print

print "\t@Override"
print "\tpublic double getEEPROMParamFloat(EEPROMParams param) {"
print "\t\tswitch (param) {"
for name in sorted(nl):
    d = offsets[name]
    if not 'type' in d:
       continue
    if not ('floating_point' in d):
       continue
    if not 'exponent' in d:
       continue
    exp = int(d['exponent'])
    s = ''
    if exp < 0:
       s = ' / %s.0d' % pow(10,-exp)
    elif exp > 0:
       s = ' * %s.0d' % pow(10, exp)
    t = d['type']
    if t == 'B':
       print "\t\tcase %s : return (double)getUInt8EEPROM(JettyEEPROM.%s)%s;" % (name.ljust(26), name, s)
       del nl[name]
    elif t == 'b':
       print "\t\tcase %s : return (double)getInt8EEPROM(JettyEEPROM.%s)%s;" % (name.ljust(26), name, s)
       del nl[name]
    elif t == 'I':
       print "\t\tcase %s : return (double)getUInt32EEPROM(JettyEEPROM.%s)%s;" % (name.ljust(26), name, s)
       del nl[name]
    elif t == 'i':
       print "\t\tcase %s : return (double)getInt32EEPROM(JettyEEPROM.%s)%s;" % (name.ljust(26), name, s)
       del nl[name]
print "\t\tdefault :\n\t\t\tBase.logger.log(Level.DEBUG, \"getEEPROMParamFloat(\" + param + \") call failed\");\n\t\t\treturn 0d;"
print "\t\t}"
print "\t}"
print ""

for k in nl:
   print "\t// Unhandled get:", k

nl = {}
for k in a.nl:
    nl[k] = a.nl[k]

print "\t@Override"
print "\tpublic int setEEPROMParam(EEPROMParams param, int val) {"
print "\t\tif (val < 0)"
print "\t\t\tval = 0;"
print "\t\tswitch (param) {"
for name in sorted(nl):
    d = offsets[name]
    if not 'type' in d:
        continue
    if 'floating_point' in d:
        continue
    t = d['type']
    if t == 'B':
       print "\t\tcase %s : setUInt8EEPROM(JettyEEPROM.%s, val); break;" % (name.ljust(26), name)
       del nl[name]
    elif t == 'b':
       print "\t\tcase %s : setInt8EEPROM(JettyEEPROM.%s, val); break;" % (name.ljust(26), name)
       del nl[name]
    elif t == 'i':
       print "\t\tcase %s : return setInt32EEPROM(JettyEEPROM.%s, val); break;" % (name.ljust(26), name)
       del nl[name]
print "\t\tdefault : Base.logger.log(Level.DEBUG, \"setEEPROMParam(\" + param + \", \" + val + \") call failed\"); break;"
print "\t\t}"
print "\t}"
print ""

print "\t@Override"
print "\tpublic long setEEPROMParam(EEPROMParams param, long val) {"
print "\t\tif (val < 0L)"
print "\t\t\tval = 0L;"
print "\t\tswitch (param) {"
for name in sorted(nl):
    d = offsets[name]
    if not 'type' in d:
        continue
    if 'floating_point' in d:
        continue
    t = d['type']
    if t == 'I':
       print "\t\tcase %s : setUInt32EEPROM(JettyEEPROM.%s, val); break;" % (name.ljust(26), name)
       del nl[name]
print "\t\tdefault : Base.logger.log(Level.DEBUG, \"setEEPROMParam(\" + param + \", \" + val + \") call failed\"); break;"
print "\t\t}"
print "\t}"
print ""

print "\t@Override"
print "\tpublic double setEEPROMParam(EEPROMParams param, double val) {"
print "\t\tif (val < 0.0d)"
print "\t\t\tval = 0.0d;"
print "\t\tswitch (param) {"
for name in sorted(nl):
    d = offsets[name]
    if not 'type' in d:
       continue
    if not ('floating_point' in d):
       continue
    if not 'exponent' in d:
       continue
    exp = int(d['exponent'])
    s = ''
    sl = ''
    sr = ''
    if exp < 0:
       sl = '('
       s = ' * %s.0d' % pow(10,-exp)
       sr = ')'
    elif exp > 0:
       sl = '('
       s = ' / %s.0d' % pow(10, exp)
       sr = ')'
    t = d['type']
    if t == 'B':
       print "\t\tcase %s : setUInt8EEPROM(JettyEEPROM.%s, (int)%sval%s%s); break;" % (name.ljust(26), name, sl, s, sr)
       del nl[name]
    elif t == 'b':
       print "\t\tcase %s : setInt8EEPROM(JettyEEPROM.%s, (int)%sval%s%s); break;" % (name.ljust(26), name, sl, s, sr)
       del nl[name]
    elif t == 'I':
       print "\t\tcase %s : setUInt32EEPROM(JettyEEPROM.%s, (long)%sval%s%s); break;" % (name.ljust(26), name, sl, s, sr)
       del nl[name]
    elif t == 'i':
       print "\t\tcase %s : setInt32EEPROM(JettyEEPROM.%s, (int)%sval%s%s); break;" % (name.ljust(26), name, sl, s, sr)
       del nl[name]
print "\t\tdefault : Base.logger.log(Level.DEBUG, \"setEEPROMParam(\" + param + \", \" + val + \") call failed\"); break;"
print "\t\t}"
print "\t}"
print ""

for k in nl:
   print "\t// Unhandled set:", k

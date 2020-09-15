from os import path, walk
import SCons.Errors

Import('env')

env.Replace(LIBS=[])
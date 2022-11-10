import os

def get_icon():
  this_dir = os.path.dirname(os.path.realpath(__file__))
  return os.path.join(this_dir, 'icons', 'icon.png')

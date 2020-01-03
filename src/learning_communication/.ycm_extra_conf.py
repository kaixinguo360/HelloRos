import os
base = os.path.dirname(os.path.abspath(__file__))

def Settings( **kwargs ):
  return {
    'flags': [ '-Wall', '-Wextra', '-Werror',
        '-I', '/opt/ros/kinetic/include',
        '-I', base + '/include',
        '-I', base + '/../../devel/include',
        ],
  }

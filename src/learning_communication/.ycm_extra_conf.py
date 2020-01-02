def Settings( **kwargs ):
  return {
    'flags': [ '-Wall', '-Wextra', '-Werror',
        '-I/opt/ros/kinetic/include',
        '-Iinclude',
        ],
  }

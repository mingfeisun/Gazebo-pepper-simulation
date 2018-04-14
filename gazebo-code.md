## Gazebo source code

Code structure 
```bash
.
├── cmake-build-debug
│   ├── CMakeFiles
│   └── msgs
├── common
├── gui
│   ├── building
│   ├── fonts
│   ├── images
│   ├── model
│   ├── plot
│   ├── qgv
│   ├── qtpropertybrowser
│   ├── terrain
│   └── viewers
├── msgs
│   └── generator
├── physics
│   ├── bullet
│   ├── dart
│   ├── ode
│   └── simbody
├── rendering
│   ├── deferred_shading
│   ├── selection_buffer
│   └── skyx
├── sensors
├── test
├── transport
└── util
```


* server main
```C++
server.reset(new gazebo::Server());
if (!server->ParseArgs(argc, argv))
  return -1;
server->Run();
server->Fini();
```

* Server class
```C++

```

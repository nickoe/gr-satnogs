# gr-satnogs: SatNOGS GNU Radio Out-Of-Tree Module
gr-satnogs is an out-of-tree GNU Radio module that provides all the necessary tools
for decoding signals from various scientific and academic sattelites.

## Install

### Requirements
* GNU Radio ( > 3.7.7 )
* CMake ( > 3.1)
* G++ (with C++11 support)
* VOLK
* libogg
* libvorbis
* libpng
* libpng++
* git

**Optional**
* gr-osmocom (for using the flowgraphs with real SDR hardware)
* libfec (it will automatically installed if not present)

#### Debian / Ubuntu
```
apt install -y build-essential cmake gnuradio g++    \
               python-mako python-six libogg-dev     \
               libvorbis-dev libpng-dev libpng++-dev
cd /tmp
git clone https://github.com/gnuradio/volk.git
cd volk
mkdir build
cd build
cmake ..
make -j 8
sudo make install
```

### Installation from source

1. `git clone https://gitlab.com/librespacefoundation/satnogs/gr-satnogs.git`
2. `cd gr-satnogs`
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`
7. `sudo make install`

If this is the first time you are building the gr-satnogs module run
`sudo ldconfig`

#### Advanced
By default, the **SatNOGS** module will use the default installation prefix.
This highly depends on the Linux distribution. You can use the `CMAKE_INSTALL_PREFIX`
variable to alter the default installation path.
E.g:

`cmake -DCMAKE_INSTALL_PREFIX=/usr ..`

Also, by default the build system enables a set of blocks used for debugging
during the development. The enable/disable switch is controled through the
`INCLUDE_DEBUG_BLOCKS` boolean variable. If for example, you want to disable the
debugging blocks, the **CMake** command would be:

`cmake -DINCLUDE_DEBUG_BLOCKS=OFF ..`

Another common control option is the library sugffix of the Linux distribution.
There are distributions like Fedora, openSUSE, e.t.c that the their 64-bit version
use the `lib64` folder to store the 64-bit versions of their dynamic libraries.
On the other hand, distributions like Ubuntu do the exact opposite. They use
`lib` directory for the libraries of the native architecture and place the 32-bit versions
on the `lib32` directory. In any case the correct library directory suffix
can be specified with the `LIB_SUFFIX` variable. For example:

`cmake -DLIB_SUFFIX=64 -DCMAKE_INSTALL_PREFIX=/usr -DINCLUDE_DEBUG_BLOCKS=OFF ..`

will install the libraries at the `/usr/lib64` directory.

## Website
For more indormation about SatNOGS please visit our [site](https://satnogs.org/).

## Release Policy
The `gr-satnogs` OOT module uses the `major.api-compatibility.minor`
versioning scheme.
is used by the [satnogs-client](https://gitlab.com/librespacefoundation/satnogs/satnogs-client), so the release and versioning policy is based on how the
satnogs client is affected by the changes on the `gr-satnogs` software.

* Minor changes, bug fixes or improvements that do not affect in anyway
the `satnogs-client` advance the `minor` version.
* The `api-compatibility` indicates changes that require modifications on `satnogs-client` but do not brake the backwards compatibility (e.g a new satallite decoder). In other words,
the `satnogs-client` should continue to operate normally without any modifications.
Changes on `satnogs-client` should be performed only to integrate the new features.
* `major` version advances when the changes break backwards compatibility with
the `satnogs-client`.

For every release change a tag with the corresponding version is created.
Releases can be retrieved by the [tags](https://gitlab.com/librespacefoundation/satnogs/gr-satnogs/tags) page.

## License

&copy; 2016,2017,2018 [Libre Space Foundation](http://librespacefoundation.org).

Licensed under the [GPLv3](LICENSE).

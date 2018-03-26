# Hexabitz Third-party Code #

**This repository contains thridparty code required for Hexabitz modules.** The thirdparty code is provided various vendors (ARM, ST, etc.) and contains both basic drivers and middleware. Please check each third-party source code for its own license. All required third-party repositories are included in this submodule to guarantee the consistancy and ease-of-use of Hexabitz module source code. We suggest you use the third-party code in this submodule instead of downloading a new copy from the vendors since the sobmodule code is already tested and guaranteed to work with module source code.

Current third-party code used in Hexabitz modules:

- ARM CMSIS V2.2.2
- STM32F0 HAL V1.3.0
- FreeRTOS V8.2.1
- FatFs R0.13

**Note: This is a git submodule repository and can not be compiled or executed on its own. It is used inside Hexabitz module repositories. Check the list of module repositories at the end.**

===============================================

## About Hexabitz ##

Hexabitz is a new kind of electronic prototyping platforms with game-changing modularity and a hint of biomimicry. Hexabitz modules connect with each other using a novel edge-soldering system and allow you to build tidy, compact and completely re-configurable electronic boards. **Learn more about Hexabitz [here](https://www.hexabitz.com/)**.

===============================================

## Useful Links ##

- Check Hexabitz main [website](https://www.hexabitz.com/) and available and planned [modules](https://www.hexabitz.com/modules/).
- Read the intro to Hexabitz modular prototyping platform on [Hackaday.io](https://hackaday.io/project/76446-hexabitz-modular-electronics-for-real)
- Overview of hexabitz software [architecture](https://hackaday.io/project/76446-hexabitz-modular-electronics-for-real/log/117213-hexabitz-software-architecture)
- Check and in-depth overview of Hexabitz code in this series of [article]().
- Hexabitz demo [projects](https://hackaday.io/list/87488-hexabitz-projects)

===============================================

## Software FAQ ##

### Q: ###
A:

Check our [website](https://www.hexabitz.com/faq/) for more information or contact us about any questions or feedback!

===============================================

## See also these module repositories ##

- [H01R00](https://bitbucket.org/hexabitz/h01r0) - RGB LED Module

- [H1BR60](https://bitbucket.org/hexabitz/h1br6) - Micro-SD Memory Card Module

- [H09R00]() - 600VAC / 1.2A Solid State Relay Module

===============================================

## Who do I talk to? ##

* Email us at info@hexabitz.com for any inquiries
* Or connect with us on [Hackaday.io](https://hackaday.io/Hexabitz)

## How do I contribute? ##

* We welcome any and all help you can provide with testing, bug fixing and adding new features! Feel free to contact us and share what's going on in your mind.
* Please send us a pull request if you have some useful code to add!

===============================================

## License ##
This software / firmware is released with [MIT license](https://opensource.org/licenses/MIT). This means you are free to use it in your own projects/products for personal or commercial applications. You are not required to open-source your projects/products as a result of using Hexabitz code inside it.

To our best knowledge, all third-party components currently included with Hexabitz software follow similar licenses (MIT, modified GPL, etc.). We will do our best to not include third-party components that require licensing or have restricted open-source terms (i.e., forcing you to open-source your project). There is no guarantee, however, that this does not happen. If we ever include a software component that requires buying a license or one that forces restrictive, open-source terms, we will mention this clearly. We advise you to verify the license of each third-party component with its vendor. 

## Disclaimer ##
HEXABITZ SOFTWARE AND HARDWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE AND HARDWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE AND HARDWARE.

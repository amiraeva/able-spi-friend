# able-spi-friend
Rust library for the Adafruit BLE SPI Friend [WIP]

Very rough around the edges (i.e. everything is hardcoded cause I'm an amateur). So far only works with the Adafruit Feather M0 BLE board, only using the Nordic UART service, with some odd timing bugs that _occasionally_ halt my RTIC project on startup or when receiving a massive bolus of packets. Use at your own risk.

Ideally the code would be cleaned up and depend on `embedded_hal` traits for the CS/IRQ pins, and the SPI bus, and export an implementation of some UART trait.

I'm not an employee of Adafruit or otherwise. This is NOT an official library/project for this device. Again, use at your own risk.



THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

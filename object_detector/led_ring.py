from time import sleep

from rpi_ws281x import Color, PixelStrip

LED_COUNT = 15
LED_PIN = 10
LED_BRIGHTNESS = 255
LED_CHANNEL = 0


def initStrip():
    """Init the strip with a global var"""

    global strip
    strip = PixelStrip(
        LED_COUNT, LED_PIN, brightness=LED_BRIGHTNESS, channel=LED_CHANNEL
    )

    strip.begin()


def clearStrip():
    """Set to black all led of the strip"""

    for i in range(LED_COUNT):
        strip.setPixelColor(i, Color(0, 0, 0))
    strip.show()


def startupBlink():
    """Blink 3 times with white color"""

    clearStrip()

    for _ in range(3):
        for i in range(LED_COUNT):
            strip.setPixelColor(i, Color(255, 255, 255))
        strip.show()

        sleep(0.5)

        clearStrip()

        sleep(0.5)


def detectionBlink():
    """Blink 3 times with red color"""

    clearStrip()

    for _ in range(3):
        for i in range(LED_COUNT):
            strip.setPixelColor(i, Color(255, 0, 0))
        strip.show()

        sleep(0.5)

        clearStrip()

        sleep(0.5)


def matchSonar(keep_going: bool):
    """Green sonar / rotating trail effect

    Args:
        keep_going: Stop the sonar or start it
    """
    clearStrip()

    trail_head = 0

    while keep_going:
        for i in range(LED_COUNT):
            pos = (trail_head - i) % LED_COUNT
            factor = (1 - (i / (LED_COUNT - 1))) ** 2.2
            green = int(255 * factor)

            strip.setPixelColor(pos, Color(0, green, 0))
            strip.show()

        trail_head = (trail_head + 1) % LED_COUNT


def endBreath():
    """Breath a blue color"""

    clearStrip()

    while True:
        for i in range(LED_COUNT):
            for j in range(LED_COUNT):
                factor = (1 - (i / (LED_COUNT - 1))) ** 2.2
                blue = int(255 * factor)

                strip.setPixelColor(j, Color(0, 0, blue))
                strip.show()
            sleep(0.05)

        for i in range(LED_COUNT):
            for j in range(LED_COUNT):
                factor = (i / (LED_COUNT - 1)) ** 2.2
                blue = int(255 * factor)

                strip.setPixelColor(j, Color(0, 0, blue))
                strip.show()
            sleep(0.05)

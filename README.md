# 99484A 2025-26 season code

[![Made with vexide badge](https://img.shields.io/badge/Made%20with-vexide-e6c85c.svg?style=for-the-badge&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAAAIGNIUk0AAHomAACAhAAA+gAAAIDoAAB1MAAA6mAAADqYAAAXcJy6UTwAAAHmUExURQAAAOXIW+bIXObIXNfb6dfb6djb6djb6eXIXOXIXObIXObIXObIXOXIXNjb6djb6djb6djb6eXIW+XIW+bIXObIW+bIW+bIW+bHW9jb6djb6djb6Njb6OXHXObIXObIXObHXObIW+XIW+XIW+bHXObHW9jb6Njb6dja6Nfb6uXHXOXHXObIXObIXObIXOXIW+bIW+bHXObHW+bIW9fb79jb6Njb6djb6djb6Njb6eXIXOXHXObIXObIXObHXObHW+bHXObIW+bIW9fb6djb6dja6Nfa6eXHXOXIW+bIW+bIW+bHW+bHXObHXObHXObIXObIXObIW+bHW+bIW+bHW+bHXObIW+bIXObIXOXHXOXHW+bHXOXIXOXHXOXHXObHXObHXObIW+XHW+bHXObIW+XHXObHXOXIXObIXObHW+bHW+XIXOXIXObIXObIW+bIW+bIXObIW+bIXObIW+bIW+bIXObIW+bIXOXIXObIXOXIW+bIW+bIW+bIW+bIXObHXOXHXOXHXObHXObIXObIXOXIW+XIW+XIXOXHXOXHXObHXObHXObHXObIXObIXObIXObIXObHXObIW+bIW+XHXObHXOXHXObIXObIXObIXObIXObIXObIW+bIXObHXObIXNjb6QAAAKYhk3UAAACfdFJOUwDh5EMBmudIQ+Fs7NM3PtzYMzbTAmLv0TRG4u1GNNHvAwNn8c4xUO5hAjDO8WcDBGvyyy4BWe3xZQMty/IEBHD0yCsDVloEKsj0cAQFdPXEKCjEdAUGePbBJSWYB333vSEhvcUlCIH5yL0jCYX6uiAKivjVth4Mjvv7jhmJ+rIbDZL8/JINC437rRwOl/2XDv66EJqaEBBui4p7Fa6uFUfmhQMAAAABYktHRACIBR1IAAAACXBIWXMAAA7DAAAOwwHHb6hkAAAAB3RJTUUH6QEaBTUsaXkSHwAAAURJREFUOMtjYBgZgHE+CDAxMzCwsC4AATZ2sDgHE1iCk4GLG8zg4WVg4OMHqxAQBMoLCYOFubkYRETFwExxCQYGSSmQAmkZBgZZObCgvKgCA4OikjKYo6LKwKDGtmCBuoYmg5Y2WEhHVw9km76BIZhrZMxgYmpmbmHJYGUNFrDhsoW4087eASzg6MTg7OLqxuDuAeZ6ennDfOLj6wcW8g8A8QKDwBy/4BCEX0PDwsGCEZEMDFERYOb8aB/k0IiJjQOLxickJkHk5yenoIRXalo6WDgjYz4MZGahqMjOyZ2PBvLyUVQUFMJVFBUXgemSUhQVZeUVEPnKquqaWjCrrh5FRUNjE0i0uaWVoa29A6yiswtFRXdP7/z5vX39YMXNIAUTJqLG/aTJvb1TpoKZ06bPmDlz5oxZaKlj9py58wY0eQ5aAACvpKcarQF0GAAAABl0RVh0U29mdHdhcmUAd3d3Lmlua3NjYXBlLm9yZ5vuPBoAAAAASUVORK5CYII=)](https://vexide.dev)
[![Contact us button](https://img.shields.io/badge/Contact%20us-c33e2e.svg?style=for-the-badge)](https://github.com/orgs/doxa-robotics/discussions/new?category=general)

This repository contains the source of 99484A's competition code for the 2025-26 school year. We use `vexide`, a new ~~experimental~~ relatively stable completely open source framework written in Rust, with our programmers actively contributing to its development on their own time.

Most of the business logic/math/theory is extracted into a separate library, called `libdoxa` which is linked from our org page.

This year, we plan to compete in:

- [First Scrimmage of the Season (V5RC MS/HS)](https://www.robotevents.com/robot-competitions/vex-robotics-competition/RE-V5RC-25-1764.html) (Christian Academy in Japan)
- [Push Back At CAJ (V5RC MS/HS)](https://www.robotevents.com/robot-competitions/vex-robotics-competition/RE-V5RC-25-1904.html) (Christian Academy in Japan)
- [2026 ASIJ VEX Robotics Tournament V5 High School Event](https://www.robotevents.com/robot-competitions/vex-robotics-competition/RE-V5RC-25-0442.html) (American School in Japan)
- [Japan V5RC HS National Regional Championship](https://www.robotevents.com/robot-competitions/vex-robotics-competition/RE-V5RC-25-0387.html) (青少年 STEM 教育振興会)
- If we qualify, VEX Worlds 2026

## About Us

We are Christian Academy in Japan's A team, composed of 11th and 12th graders (高校２〜３年生). For more information, see our [GitHub org profile](https://github.com/doxa-robotics/vexide).

## Repository Structure

This repository is organized as follows:

- **`/src`**: Source code for robot control, autonomous routines, and other software components.
  - **`/autons`**: Files for each autonomous route. Each route runs for 15 seconds, with the exception of `skills.rs`, which lasts 60 seconds.
  - **`/opcontrol`**: Control for the teleop phase.
  - **`/subsystems`**: Subsystem control for this year's custom subsystems.
  - **`/utils`**: Various utilities, including a custom logger which logs to the SD card.

## License

At the end of the season, this project will be relicensed under the MIT License. However, for the time being, this code is _copyrighted_. All rights reserved.

## Contact

For questions, suggestions, or other inquiries, please DM us at [@doxarobotics](https://www.instagram.com/doxarobotics) or create a new GitHub discussion.

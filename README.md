# Orion II CanSat

## Who are we?
We are a team composed of 12 students of the [Model High School “Evangeliki Scholi Smyrna”](http://lyk-evsch-n-smyrn.att.sch.gr/wordpress/?p=1322). For the fourth consecutive time we are participating in the Panhellenic competition "[CanSat in Greece](https://cansat.gr/)", a qualifier of the Pan-European competition "[CanSats in Europe](http://www.esa.int/SPECIALS/CanSat/SEMXTDCKP6G_0.html)", organized by ESA (European Space Agency) for the last 8 years. Our team is called Orion II and our supervisor is Dr. Christos Fanidis.

&nbsp;
## Our Inspiration

&nbsp;
## Do you want to contibute?
Contibuting is a great way to help a project gain traction and take off. First and foremost you can fork the repository and start working on your own. Then you can create a pull request to merge it to our meain repository, or you can create your custom spin off of your programs. Beware you must be compilient with your license, AGPLv3.

&nbsp;
## Meet our Team (In alphabetiac order)
|             Name             |                                   Role                                   |              Introduction              |
| ---------------------------- | ------------------------------------------------------------------------ | -------------------------------------- |
|        __Danae Veli__        |                  Head of Electrical Engineering                          |                                        |
|    __Dennis Angelopoulos__   |      Head of Mechanical Engineering and Control Systems Development      |                                        |
|     __Dimitris Balatos__     |                     Head of Software Engineering                         |                                        |
|     __Giorgos Kaltsis__      |                         Senior Data Analyst                              |                                        |
| __Konstantinos Kritharidis__ |                    Head of Recovery Systems                              |                                        |
| [__Rikarnto Bariampa__](https://github.com/richardbar) | Head of Telemetry and Senior Software Engineer |                                        |

&nbsp;
## Want to reach us?
If you have any question, please feel free to reach us in one of the following ways:
* [__Email__](mailto:orioncantgr@gmail.com)
* [__Facebook__](https://www.facebook.com/orioncansatteam)
* [__Twitter__](https://twitter.com/OrionCanSat2020)

&nbsp;
## F.A.Q.
---
### __Q: Why did you make v0.4 and v0.5 obsolete?__
### __A:__ We chose to make the v0. 4 and v0.5 obsolete due to some problems with the development of some of the CanSat's subsystems. Some of those subsystems included the Memory Management System (__MMS__), the first version of the Automatic Flight Controller (__AFC__) and the buffers that would've been used for the storage and sending of the data collected. Those problems lead us to make the v0.4 and v0.5 obsolete. Some of the source code of those subsystems did not make it to GitHub while other parts of the source code where not committed and or pushed. We are committed to re-introducing those features in version v0.6 and v0.7 and polish them for the v0.8.
&nbsp;
### __Q: Why did you choose to not include a Universal RF module?__
### __A:__ The [RadioHead Library](http://www.airspayce.com/mikem/arduino/RadioHead/) provides a good, simple and coherent way to communicate with a wide range of RF modules. That is why in our code we will be using the RadioHead Library instead of making an inhouse compatibility layer which will be time consuming and is generally not worth it.
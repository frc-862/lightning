# lightning

![Build](https://github.com/frc-862/lightning/workflows/Build/badge.svg)
[![Build Samples](https://github.com/frc-862/lightning/actions/workflows/sample-build.yml/badge.svg)](https://github.com/frc-862/lightning/tree/develop/samples)\
[![Javadoc](https://github.com/frc-862/lightning/actions/workflows/javadoc.yml/badge.svg)](https://frc-862.github.io/lightning/)
[![GitHub Package Registry Version](https://img.shields.io/github/release/frc-862/lightning.svg?label=GPR%20Version&logo=github&color=orange)](https://github.com/frc-862/lightning/packages/1123041)

## Description

This repository contains base code for Team 862's FRC robots.

## Getting Started

To add the library to a WPILib Robot Project, make sure the project has all the necessary files in `vendordeps/` as seen [here](https://github.com/frc-862/lightning/tree/master/vendordeps).

You must also set up the following dependency repository in your `build.gradle`.

```groovy
repositories {
    maven {
        name = "GitHubPackages"
        url = "https://maven.pkg.github.com/frc-862/lightning"
        credentials {
            username = project.findProperty("gpr.user")
            password = project.findProperty("gpr.key")
        }
    }
}
```

You should set your `~/.gradle/gradle.properties` (Unix) or `%HOMEPATH%\.gradle\gradle.properties` (Windows) file to contain your username as well as a GitHub Personal Access Token

```groovy
gpr.user=<your-username>
gpr.key=<your-personal-access-token>
```

Note that you do not need to be a member of the [@frc-862](https://github.com/frc-862) organization to have access to this project.

Lastly, you will need to add the library as a dependency as shown below

```groovy
dependencies {
    implementation 'com.lightningrobotics:common:<version>'
}
```

The latest version of this repository (for `<version>` above) is [![Latest Version](https://img.shields.io/github/release/frc-862/lightning.svg?label=version&color=blue)](https://github.com/frc-862/lightning/packages/1123041).

## Contributing

### Releases

To publish a new version, [this action](https://github.com/frc-862/lightning/actions/workflows/deploy.yml) must be run.
The action requires the desired version number and release title.

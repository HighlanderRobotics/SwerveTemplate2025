# 8033 2025 Template

This repo provides a starting point for 8033's 2025 code.
The goal is to clean up and port our 2024 swerve and vision code to WPILib 2025 beta.
This is NOT our 2025 Reefscape repo because WPILib does not support in place upgrading from betas to the kickoff release.

The following [slack post](https://frc8033.slack.com/archives/CJNQ8JYE6/p1733968285750289) summarizes the goals for the repo:
```
I'm starting work on our 2025 swerve and vision code. this is mainly a port/cleanup of our 2024 code to get rid of season-specific features. this is not going to be on our 2025 repo ("Reefscape") because WPILib does not support in-place upgrades from 2025 betas to the 2025 kickoff release. There is probably a way to work around that, but I dont want to figure that out and it should be easy enough to copy over relevant code to a new repo. if someone feels differently about having a separate repo im open to figuring out an in place upgrade later (iirc the kickoff release is scheduled for dec 30 so there should be a few days to figure it out?). other main changes i want to impl:
1) a way to switch between different robot constants (ie Banshee, Alpha, Comp) to allow testing on different robots
2) improved swerve sim either with maplesim or a custom bit of swerve math. my goal with improved swerve sim is to better account for the dynamics of the full robot's weight rather than just the moi of each swerve module independently, maplesim also provides collision sim but imo thats not worth the effort either.
```

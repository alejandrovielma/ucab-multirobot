# 0003. A repository for our collector only

**Status:** accepted (05/10/2026)

## Context
There are several robots on the track, but each one is a separate project by another team. Our team of 4 builds a single collector robot.

## Decision
This repo contains only our robot. There is no code for the other robots or for coordinating a fleet. Anything that has to be agreed with the other teams is recorded in `docs/external-interface.md`.

## Consequences
- The repo is smaller and everyone knows what they are responsible for.
- If ROS communication with the other robots is agreed, the shared messages are added as an agreed dependency, without copying their code.

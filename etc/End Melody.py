import random  # Importing the random library for generating random colors

async def sing(robot):
    star_trek_intro_melody = [
    (Note.E4, Note.QUARTER),   # E4
    (Note.G4, Note.HALF),      # G4
    (Note.C5, Note.HALF),      # C5
    (Note.B4, Note.QUARTER),   # B4
    (Note.C5, Note.QUARTER),   # C5
    (Note.E4, Note.HALF),      # E4
    (Note.G4, Note.WHOLE),     # G4

    (Note.G4, Note.HALF),      # G4
    (Note.A4, Note.QUARTER),   # A4
    (Note.C5, Note.QUARTER),   # C5
    (Note.B4, Note.HALF),      # B4
    (Note.A4, Note.QUARTER),   # A4

    (Note.B4, Note.WHOLE),     # B4
    (Note.C5, Note.HALF),      # C5
    (Note.B4, Note.QUARTER),   # B4

    (Note.G4, Note.QUARTER),   # G4
    (Note.E4, Note.WHOLE),     # E4

    (Note.A4, Note.WHOLE),     # A4
    (Note.G4, Note.WHOLE),     # G4

    (Note.E4, Note.HALF),      # E4
    (Note.C4, Note.WHOLE)      # C4
]

    await robot.set_wheel_speeds(speed, -speed)

    for note in star_trek_intro_melody:
        await robot.play_note(note[0], note[1])

        # Generate random RGB values for lights for every note 
        r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        await robot.set_lights_spin_rgb(r, g, b)

    await robot.set_wheel_speeds(0, 0)
    pose = await robot.get_position()  # Get the current position
    current_heading = pose.heading     # Extract the heading (angle)
    turn_angle = -current_heading -90     # Calculate the shortest way to turn back to 0 degrees
    await robot.turn_left(turn_angle)
    await robot.set_lights_blink_rgb(0, 255, 0)
    print(await robot.get_position())

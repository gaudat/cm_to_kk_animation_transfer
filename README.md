Transfer COM3D2 anm files to Koikatsu animation rig.

Basically copying pose of corresponding bones between rigs.

KK rig copied from the blender project included in `johnbbob_la_petite` and `oscar`'s Animation Guide found on KK Discord.

bl_28 branch of [CM3D2 converter]("https://github.com/CM3D2user/Blender-CM3D2-Converter") is needed to import anm files.

The converter does not work in 2.93. To fix this, change all declarations of bpy.props in operator class definitions from using `=` to `.

Read `main.py` for the complete script. The script is organized from low-level to high-level.
Read `TransferPoseCommon` class and onwards if you want to use it.

Transform calculations are done by trial and error. I wasn't that bad at linear algebra back in college lol.

You will need to calculate a new T-pose basis for new rigs. See `SaveTPoseBasisCommon` on generating `tpose_basis.json`.

I wanted to transfer COM3D2's positions into KK at first. But it is too difficult as the animation clips do not match between the two. I am done with this for now. Maybe someone can follow up on making studio animation zipmods from this. But isn't it better to use VMD as a common format instead?
# Small framework in place
(Note this is not to be compared to an actual javascript framework)

Originally all these files were based in draw-general.
All the (bottom) menu tabs should have a file in this folder.
Every such file(except actual-objects and general), called for example draw-trajectories, has an id. The id is trajectories in this case of menu-trajectories.java
Then this file must have the following functions:
- drawTableSelected{Id}
- removeTableDrawn{Id}
- removeSpecificTableDrawn{Id - plural}
- drawHover{Id - plural}
- removeHover{Id}

Note that if you add a file, you have to add the new file(and it's functions) to the following functions:
- removeHover()
- createTables()
- executeTableDraws()
- removeTableDraws()
- doSelectAllOfTable()
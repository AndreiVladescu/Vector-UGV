# Hardware

One KiCad project per board: `side-board/`, `power-board/`, `carrier-cm5/`. The lib tables in each point to the shared library in `lib/`, so custom symbols and footprints go there.

- Connector pinouts come from `docs/interfaces.md`. Use the same footprint and pin order on both ends of a cable.
- Side board: draw the leg cell as one hierarchical sheet, use it three times, and copy the layout with KiCad's multichannel tools.
- Tag each fabbed revision (`side-board-revA`) and put the revision and git hash on the silkscreen. Gerbers, BOM and pick-and-place go in `<board>/production/<rev>/`.
- Hand assembly: 0402 minimum, one side where possible, test points and 0 Ω links on every rail.

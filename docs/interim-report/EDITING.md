# Editing the interim report

The report's text lives in `sections/`, one Markdown file per section. The page (`index.html`)
puts them together when it loads, numbers the figures and tables, and builds the contents list.
You never need to touch `index.html` or `report.js` to change the text.

| File | Section | Written by |
| --- | --- | --- |
| `00-summary.md` | Executive Summary | Team |
| `01-introduction.md` | 1. Introduction | Team |
| `02-background.md` | 2. Background | Team |
| `03-requirements.md` | 3. Requirements and Specifications | Team |
| `04-concept.md` | 4. Concept Development | Team |
| `05-planner.md` | 5. VATEL Planner | Teh Wei Sheng |
| `06-helmet.md` | 6. Sensor Helmet | Teh Wei Sheng |
| `07-navigation.md` | 7. Navigation and Localisation | Soh Sze Juin |
| `08-manipulation.md` | 8. Plate Perception and Manipulation | Song Cheng Yan |
| `09-plan.md` | 9. Project Plan | Team |
| `10-contributions.md` | 10. Individual Contributions | Team |
| `11-conclusion.md` | 11. Conclusion | Team |
| `references.md` | References | |
| `appendices.md` | Appendices | |

Edit only your own files where you can, so two people never change the same file at once.

## Editing on github.com

1. Open the file in the repository: `docs/interim-report/sections/<file>.md`.
2. Click the pencil icon (Edit this file). Use the **Preview** tab to check it.
3. Click **Commit changes**. The published report updates about a minute later.

## Writing Markdown

```markdown
## 7. Autonomous Navigation and Localisation      <- the section heading (one per file)
### 7.1. Overview                                  <- a subsection; appears in the contents list

A paragraph is plain text. Leave a blank line between paragraphs.
**bold**, *italic*, `code`, [a link](https://example.com)

- a bullet
1. a numbered item
```

### The header block

Every file starts with a header that names its author. It shows under the section's heading.

```markdown
---
author: Soh Sze Juin
---
```

### Figures

Put the image in `docs/assets/interim/` (WebP, PNG or JPG, about 2000 px wide at most), then:

```markdown
![fig:costmap | The dynamic costmap: obstacle points decay once a person has left](../assets/interim/costmap.webp)
```

`fig:costmap` is the figure's label and the text after `|` is its caption. Figures are numbered
automatically, so refer to them by label in the text and the number fills itself in:

```markdown
The costmap in [[fig:costmap]] shows ...      ->  "The costmap in Figure 7 shows ..."
```

### Tables

Write a Markdown table, with a caption line directly above it:

```markdown
Table: tab:nav-tests | Table approach tests
| Run | Room | Stop distance (cm) |
| --- | --- | --- |
| 1 | Sandbox | 30.4 |
```

Refer to it as `[[tab:nav-tests]]`, which becomes "Table 5".

### Notes for the team

```markdown
> TODO: add the number of runs here.
```

This shows as a yellow "To fill" box and is not counted as words. Delete it once it is done.

### Citations

APA 7th edition: cite in the text as (Author, Year), for example (Macenski et al., 2020), and add
the full reference to `references.md` in alphabetical order.

## Word count

The limit is 5,000 words, excluding captions, references and appendices. Add `?wc` to the
report's address to see each section's count and the total:

`https://cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/interim-report/?wc`

## Rules

- **Never name the industry partner or its venue.** Write "our industry partner". The site is
  public, and the project is under an NDA.
- Use only footage and photos that the people and venues in them have agreed to show.

## Previewing on your own computer

Opening `index.html` straight from the disk does not work: the browser will not let it read the
section files. Serve the `docs/` folder instead and open the report from there:

```bash
python -m http.server 8091 --directory docs
```

Then go to http://localhost:8091/interim-report/.

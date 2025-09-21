Compositors
===========

Terminology
----------
- Visual Compositor: A takes in a structured pacenote and produces the parameters for showing a visual pacenote.
- Text Compositor (sometimes just Compositor): A takes in a structured pacenote and produces an array of strings representing the pacenote.
- Sub Phrase (or Phrase): A single string in the TextCompositor array output. Refers to a single sub-part of the pacenote.

Text Compositors
----------------
This folder is organized like so:

.../compositors/<compositor_name>/compositor.lua
.../compositors/<compositor_name>/<voicepack>/...

1. A compositor name, often named after a language, such as english, spanish, etc.
2. A voicepack, such as British Male, Spanish Female, etc. Voices are derived from the Text-to-Speech backend's available voices.

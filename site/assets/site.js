"use strict";

function enableCopy() {
  if (!navigator.clipboard?.writeText) return;
  for (const code of document.querySelectorAll("pre > code")) {
    const pre = code.parentElement;
    const wrapper = document.createElement("div");
    wrapper.className = "command";
    pre.replaceWith(wrapper);
    wrapper.append(pre);

    const button = document.createElement("button");
    button.className = "copy-button";
    button.type = "button";
    button.textContent = "Copy";
    button.setAttribute("aria-label", "Copy command");
    button.addEventListener("click", async () => {
      try {
        await navigator.clipboard.writeText(code.textContent);
        button.textContent = "Copied";
        button.setAttribute("aria-label", "Command copied");
      } catch {
        const range = document.createRange();
        range.selectNodeContents(code);
        const selection = window.getSelection();
        selection.removeAllRanges();
        selection.addRange(range);
        button.textContent = "Selected";
        button.setAttribute("aria-label", "Command selected for copying");
      }
    });
    wrapper.append(button);
  }
}

enableCopy();

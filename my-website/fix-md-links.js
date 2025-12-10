// fix-md-links.js
// Run with: node fix-md-links.js

const fs = require('fs');
const path = require('path');

// Path to your docs folder
const DOCS_DIR = path.join(__dirname, 'docs');

function fixLinksInFile(filePath) {
  let content = fs.readFileSync(filePath, 'utf-8');
  // Regex to match Markdown links: [text](link.md)
  const regex = /\[([^\]]+)\]\(([^)]+)\.md\)/g;

  const fixedContent = content.replace(regex, (_, text, link) => {
    return `[${text}](${link})`; // remove .md extension
  });

  if (fixedContent !== content) {
    fs.writeFileSync(filePath, fixedContent, 'utf-8');
    console.log(`Fixed links in: ${filePath}`);
  }
}

function traverseDir(dir) {
  const files = fs.readdirSync(dir);
  files.forEach(file => {
    const fullPath = path.join(dir, file);
    const stat = fs.statSync(fullPath);
    if (stat.isDirectory()) {
      traverseDir(fullPath);
    } else if (fullPath.endsWith('.md')) {
      fixLinksInFile(fullPath);
    }
  });
}

// Start fixing
traverseDir(DOCS_DIR);
console.log('All Markdown links fixed!');

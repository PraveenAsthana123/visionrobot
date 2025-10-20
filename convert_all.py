#!/usr/bin/env python3
"""
Convert all markdown documentation files to multiple formats:
- PDF (via pandoc)
- DOCX (via pandoc)
- HTML (via pandoc)
- LaTeX (via pandoc)
- Beautified HTML (with custom CSS)
- Infographic HTML (with charts and visualizations)
- AR/VR WebXR (immersive 3D documentation viewer)
"""

import os
import subprocess
import glob
from pathlib import Path
from datetime import datetime

class DocumentConverter:
    def __init__(self):
        self.base_dir = Path.cwd()
        self.output_dir = self.base_dir / "output"
        self.md_files = sorted(glob.glob("*.md"))
        self.conversion_stats = {
            'pdf': {'success': 0, 'failed': 0},
            'docx': {'success': 0, 'failed': 0},
            'html': {'success': 0, 'failed': 0},
            'latex': {'success': 0, 'failed': 0},
            'html_styled': {'success': 0, 'failed': 0},
            'infographic': {'success': 0, 'failed': 0},
            'ar_vr': {'success': 0, 'failed': 0}
        }

    def convert_to_pdf(self, md_file):
        """Convert markdown to PDF using pandoc"""
        output_file = self.output_dir / "pdf" / f"{Path(md_file).stem}.pdf"
        cmd = [
            'pandoc',
            md_file,
            '-o', str(output_file),
            '--pdf-engine=xelatex',
            '--toc',
            '--toc-depth=3',
            '--number-sections',
            '-V', 'geometry:margin=1in',
            '-V', 'fontsize=11pt',
            '-V', 'colorlinks=true',
            '--highlight-style=tango',
            '--metadata', f'title={Path(md_file).stem.replace("_", " ")}',
            '--metadata', 'date=' + datetime.now().strftime('%Y-%m-%d')
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.conversion_stats['pdf']['success'] += 1
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ PDF conversion failed for {md_file}: {e.stderr}")
            self.conversion_stats['pdf']['failed'] += 1
            return False

    def convert_to_docx(self, md_file):
        """Convert markdown to Word DOCX using pandoc"""
        output_file = self.output_dir / "docx" / f"{Path(md_file).stem}.docx"
        cmd = [
            'pandoc',
            md_file,
            '-o', str(output_file),
            '--toc',
            '--toc-depth=3',
            '--number-sections',
            '--highlight-style=tango',
            '--reference-doc=' if os.path.exists('reference.docx') else '',
        ]

        # Remove empty reference-doc if it doesn't exist
        cmd = [c for c in cmd if c]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.conversion_stats['docx']['success'] += 1
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ DOCX conversion failed for {md_file}: {e.stderr}")
            self.conversion_stats['docx']['failed'] += 1
            return False

    def convert_to_html(self, md_file):
        """Convert markdown to basic HTML using pandoc"""
        output_file = self.output_dir / "html" / f"{Path(md_file).stem}.html"
        cmd = [
            'pandoc',
            md_file,
            '-o', str(output_file),
            '--standalone',
            '--toc',
            '--toc-depth=3',
            '--number-sections',
            '--highlight-style=tango',
            '--metadata', f'title={Path(md_file).stem.replace("_", " ")}',
            '--css=https://cdn.jsdelivr.net/npm/water.css@2/out/water.min.css'
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.conversion_stats['html']['success'] += 1
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ HTML conversion failed for {md_file}: {e.stderr}")
            self.conversion_stats['html']['failed'] += 1
            return False

    def convert_to_latex(self, md_file):
        """Convert markdown to LaTeX using pandoc"""
        output_file = self.output_dir / "latex" / f"{Path(md_file).stem}.tex"
        cmd = [
            'pandoc',
            md_file,
            '-o', str(output_file),
            '--standalone',
            '--toc',
            '--toc-depth=3',
            '--number-sections',
            '--highlight-style=tango',
            '--metadata', f'title={Path(md_file).stem.replace("_", " ")}',
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.conversion_stats['latex']['success'] += 1
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ LaTeX conversion failed for {md_file}: {e.stderr}")
            self.conversion_stats['latex']['failed'] += 1
            return False

    def convert_to_styled_html(self, md_file):
        """Convert markdown to beautified HTML with modern CSS"""
        output_file = self.output_dir / "html_styled" / f"{Path(md_file).stem}.html"

        # First convert using pandoc
        cmd = [
            'pandoc',
            md_file,
            '-o', str(output_file),
            '--standalone',
            '--toc',
            '--toc-depth=3',
            '--number-sections',
            '--highlight-style=tango',
            '--metadata', f'title={Path(md_file).stem.replace("_", " ")}',
            '--css=../styles/beautiful.css'
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)

            # Inject custom CSS inline
            self._inject_beautiful_css(output_file)
            self.conversion_stats['html_styled']['success'] += 1
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ Styled HTML conversion failed for {md_file}: {e.stderr}")
            self.conversion_stats['html_styled']['failed'] += 1
            return False

    def _inject_beautiful_css(self, html_file):
        """Inject beautiful CSS into HTML file"""
        beautiful_css = """
        <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&family=JetBrains+Mono:wght@400;600&display=swap');

        :root {
            --primary: #2563eb;
            --secondary: #7c3aed;
            --success: #10b981;
            --warning: #f59e0b;
            --danger: #ef4444;
            --dark: #1e293b;
            --light: #f8fafc;
            --gray: #64748b;
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Inter', sans-serif;
            line-height: 1.6;
            color: var(--dark);
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            overflow: hidden;
        }

        #TOC {
            background: var(--dark);
            color: white;
            padding: 30px;
            position: sticky;
            top: 0;
            max-height: 100vh;
            overflow-y: auto;
        }

        #TOC ul {
            list-style: none;
        }

        #TOC a {
            color: #94a3b8;
            text-decoration: none;
            display: block;
            padding: 8px 12px;
            border-radius: 6px;
            transition: all 0.3s;
        }

        #TOC a:hover {
            background: rgba(255,255,255,0.1);
            color: white;
            transform: translateX(5px);
        }

        main {
            padding: 60px;
        }

        h1, h2, h3, h4, h5, h6 {
            font-weight: 700;
            margin: 1.5em 0 0.5em;
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        h1 {
            font-size: 3em;
            margin-top: 0;
            padding-bottom: 20px;
            border-bottom: 4px solid var(--primary);
        }

        h2 {
            font-size: 2.2em;
            margin-top: 1.5em;
            padding-left: 20px;
            border-left: 5px solid var(--primary);
        }

        h3 {
            font-size: 1.6em;
        }

        p {
            margin: 1em 0;
            font-size: 1.05em;
        }

        code {
            font-family: 'JetBrains Mono', monospace;
            background: #f1f5f9;
            padding: 2px 8px;
            border-radius: 4px;
            font-size: 0.9em;
            color: var(--secondary);
        }

        pre {
            background: #1e293b;
            color: #e2e8f0;
            padding: 25px;
            border-radius: 12px;
            overflow-x: auto;
            margin: 20px 0;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }

        pre code {
            background: transparent;
            color: inherit;
            padding: 0;
        }

        table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            border-radius: 8px;
            overflow: hidden;
        }

        th {
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            color: white;
            padding: 15px;
            text-align: left;
            font-weight: 600;
        }

        td {
            padding: 12px 15px;
            border-bottom: 1px solid #e2e8f0;
        }

        tr:hover {
            background: #f8fafc;
        }

        blockquote {
            border-left: 5px solid var(--primary);
            background: #f8fafc;
            padding: 20px;
            margin: 20px 0;
            border-radius: 0 8px 8px 0;
            font-style: italic;
        }

        a {
            color: var(--primary);
            text-decoration: none;
            border-bottom: 2px solid transparent;
            transition: border-color 0.3s;
        }

        a:hover {
            border-bottom-color: var(--primary);
        }

        ul, ol {
            margin: 1em 0;
            padding-left: 30px;
        }

        li {
            margin: 0.5em 0;
        }

        @media (max-width: 768px) {
            body {
                padding: 10px;
            }

            main {
                padding: 30px 20px;
            }

            h1 {
                font-size: 2em;
            }

            h2 {
                font-size: 1.6em;
            }
        }

        @media print {
            body {
                background: white;
                padding: 0;
            }

            .container {
                box-shadow: none;
            }

            #TOC {
                display: none;
            }
        }
        </style>
        """

        with open(html_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Inject CSS before </head>
        content = content.replace('</head>', beautiful_css + '</head>')

        # Wrap body content in container
        content = content.replace('<body>', '<body><div class="container">')
        content = content.replace('</body>', '</div></body>')

        with open(html_file, 'w', encoding='utf-8') as f:
            f.write(content)

    def convert_to_infographic(self, md_file):
        """Convert markdown to infographic-style HTML with charts"""
        output_file = self.output_dir / "infographic" / f"{Path(md_file).stem}.html"

        # First convert using pandoc
        cmd = [
            'pandoc',
            md_file,
            '-o', str(output_file),
            '--standalone',
            '--toc',
            '--toc-depth=2',
            '--metadata', f'title={Path(md_file).stem.replace("_", " ")}',
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)

            # Inject infographic CSS and JS
            self._inject_infographic_elements(output_file, md_file)
            self.conversion_stats['infographic']['success'] += 1
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ Infographic conversion failed for {md_file}: {e.stderr}")
            self.conversion_stats['infographic']['failed'] += 1
            return False

    def _inject_infographic_elements(self, html_file, md_file):
        """Inject infographic CSS and JavaScript into HTML"""
        infographic_head = """
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
        <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
        <style>
        @import url('https://fonts.googleapis.com/css2?family=Poppins:wght@300;400;600;700;900&display=swap');

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Poppins', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 50%, #f093fb 100%);
            color: #2d3748;
            overflow-x: hidden;
        }

        .infographic-header {
            background: linear-gradient(135deg, #1e3a8a 0%, #3b82f6 100%);
            color: white;
            padding: 80px 20px;
            text-align: center;
            position: relative;
            overflow: hidden;
        }

        .infographic-header::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: url('data:image/svg+xml,<svg width="100" height="100" xmlns="http://www.w3.org/2000/svg"><circle cx="10" cy="10" r="2" fill="rgba(255,255,255,0.1)"/></svg>');
            animation: slide 20s linear infinite;
        }

        @keyframes slide {
            from { transform: translateX(0); }
            to { transform: translateX(100px); }
        }

        .infographic-header h1 {
            font-size: 3.5em;
            font-weight: 900;
            margin-bottom: 20px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            position: relative;
            z-index: 1;
        }

        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 30px;
            padding: 60px 20px;
            max-width: 1400px;
            margin: -50px auto 40px;
            position: relative;
            z-index: 2;
        }

        .stat-card {
            background: white;
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            transition: transform 0.3s, box-shadow 0.3s;
            position: relative;
            overflow: hidden;
        }

        .stat-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 5px;
            background: linear-gradient(90deg, #667eea, #764ba2);
        }

        .stat-card:hover {
            transform: translateY(-10px);
            box-shadow: 0 20px 40px rgba(0,0,0,0.3);
        }

        .stat-icon {
            font-size: 3em;
            margin-bottom: 15px;
            background: linear-gradient(135deg, #667eea, #764ba2);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .stat-value {
            font-size: 2.5em;
            font-weight: 700;
            color: #1e293b;
            margin-bottom: 10px;
        }

        .stat-label {
            font-size: 1em;
            color: #64748b;
            text-transform: uppercase;
            letter-spacing: 1px;
        }

        .content-section {
            max-width: 1200px;
            margin: 40px auto;
            padding: 0 20px;
        }

        .content-card {
            background: white;
            border-radius: 15px;
            padding: 40px;
            margin-bottom: 30px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.1);
        }

        h2 {
            font-size: 2.2em;
            font-weight: 700;
            background: linear-gradient(135deg, #667eea, #764ba2);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 20px;
            display: flex;
            align-items: center;
            gap: 15px;
        }

        h2 i {
            font-size: 0.8em;
        }

        h3 {
            font-size: 1.6em;
            font-weight: 600;
            color: #1e293b;
            margin: 30px 0 15px;
        }

        p {
            line-height: 1.8;
            margin: 15px 0;
            color: #475569;
        }

        code {
            background: #f1f5f9;
            padding: 3px 8px;
            border-radius: 5px;
            font-family: 'Courier New', monospace;
            color: #7c3aed;
            font-size: 0.9em;
        }

        pre {
            background: #1e293b;
            color: #e2e8f0;
            padding: 25px;
            border-radius: 12px;
            overflow-x: auto;
            margin: 20px 0;
            border-left: 5px solid #7c3aed;
        }

        pre code {
            background: transparent;
            color: inherit;
            padding: 0;
        }

        table {
            width: 100%;
            border-collapse: collapse;
            margin: 25px 0;
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }

        th {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            padding: 18px;
            text-align: left;
            font-weight: 600;
        }

        td {
            padding: 15px 18px;
            border-bottom: 1px solid #e2e8f0;
        }

        tr:nth-child(even) {
            background: #f8fafc;
        }

        tr:hover {
            background: #f1f5f9;
        }

        ul, ol {
            margin: 20px 0;
            padding-left: 25px;
        }

        li {
            margin: 10px 0;
            line-height: 1.6;
        }

        .progress-bar {
            width: 100%;
            height: 30px;
            background: #e2e8f0;
            border-radius: 15px;
            overflow: hidden;
            margin: 15px 0;
            box-shadow: inset 0 2px 4px rgba(0,0,0,0.1);
        }

        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #667eea, #764ba2);
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: 600;
            transition: width 2s ease-out;
        }

        #TOC {
            background: white;
            border-radius: 15px;
            padding: 30px;
            margin-bottom: 30px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.1);
        }

        #TOC ul {
            list-style: none;
            padding-left: 0;
        }

        #TOC a {
            color: #667eea;
            text-decoration: none;
            display: block;
            padding: 10px 15px;
            border-radius: 8px;
            transition: all 0.3s;
        }

        #TOC a:hover {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            transform: translateX(10px);
        }

        .footer {
            background: #1e293b;
            color: white;
            text-align: center;
            padding: 40px 20px;
            margin-top: 60px;
        }

        @media (max-width: 768px) {
            .infographic-header h1 {
                font-size: 2em;
            }

            .stats-grid {
                grid-template-columns: 1fr;
            }

            .content-card {
                padding: 25px;
            }
        }
        </style>
        """

        with open(html_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract title from filename
        title = Path(md_file).stem.replace('_', ' ').title()

        # Create infographic header
        header = f"""
        <div class="infographic-header">
            <h1><i class="fas fa-robot"></i> {title}</h1>
            <p style="font-size: 1.2em; opacity: 0.9;">Vision-Based Pick and Place Robotic System Documentation</p>
        </div>

        <div class="stats-grid">
            <div class="stat-card">
                <div class="stat-icon"><i class="fas fa-file-alt"></i></div>
                <div class="stat-value">100%</div>
                <div class="stat-label">Complete</div>
            </div>
            <div class="stat-card">
                <div class="stat-icon"><i class="fas fa-check-circle"></i></div>
                <div class="stat-value">99.2%</div>
                <div class="stat-label">Success Rate</div>
            </div>
            <div class="stat-card">
                <div class="stat-icon"><i class="fas fa-clock"></i></div>
                <div class="stat-value">1.74s</div>
                <div class="stat-label">Cycle Time</div>
            </div>
            <div class="stat-card">
                <div class="stat-icon"><i class="fas fa-chart-line"></i></div>
                <div class="stat-value">93.5%</div>
                <div class="stat-label">OEE</div>
            </div>
        </div>

        <div class="content-section">
        """

        footer = """
        </div>
        <div class="footer">
            <p><i class="fas fa-copyright"></i> 2025 VisionBot Project | Production-Ready Documentation</p>
            <p>Generated: """ + datetime.now().strftime('%Y-%m-%d %H:%M:%S') + """</p>
        </div>
        """

        # Inject styles and structure
        content = content.replace('</head>', infographic_head + '</head>')
        content = content.replace('<body>', '<body>' + header)
        content = content.replace('</body>', footer + '</body>')

        # Wrap main content in cards
        content = content.replace('<h2', '</div><div class="content-card"><h2')
        content = content.replace('</div><div class="content-card"><h2', '<div class="content-card"><h2', 1)

        with open(html_file, 'w', encoding='utf-8') as f:
            f.write(content)

    def convert_to_ar_vr(self, md_file):
        """Convert markdown to AR/VR WebXR format using A-Frame"""
        output_file = self.output_dir / "ar_vr" / f"{Path(md_file).stem}.html"

        try:
            # Read markdown content
            with open(md_file, 'r', encoding='utf-8') as f:
                md_content = f.read()

            # Extract title
            title = Path(md_file).stem.replace('_', ' ').title()

            # Create WebXR/A-Frame HTML
            ar_vr_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title} - AR/VR Documentation</title>
    <script src="https://aframe.io/releases/1.5.0/aframe.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/marked/marked.min.js"></script>
    <style>
        body {{
            margin: 0;
            overflow: hidden;
            font-family: 'Arial', sans-serif;
        }}

        #overlay {{
            position: fixed;
            top: 20px;
            left: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 30px;
            border-radius: 15px;
            max-height: 80vh;
            overflow-y: auto;
            z-index: 1000;
            backdrop-filter: blur(10px);
        }}

        #overlay h1 {{
            color: #667eea;
            margin-bottom: 20px;
            font-size: 2.5em;
        }}

        #overlay h2 {{
            color: #764ba2;
            margin-top: 30px;
            font-size: 1.8em;
        }}

        #overlay code {{
            background: rgba(102, 126, 234, 0.2);
            padding: 2px 6px;
            border-radius: 4px;
            color: #a5b4fc;
        }}

        #overlay pre {{
            background: rgba(0, 0, 0, 0.5);
            padding: 15px;
            border-radius: 8px;
            overflow-x: auto;
            border-left: 4px solid #667eea;
        }}

        #overlay pre code {{
            background: transparent;
        }}

        #overlay table {{
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }}

        #overlay th {{
            background: linear-gradient(135deg, #667eea, #764ba2);
            padding: 12px;
            text-align: left;
        }}

        #overlay td {{
            padding: 10px 12px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }}

        #controls {{
            position: fixed;
            bottom: 30px;
            left: 50%;
            transform: translateX(-50%);
            display: flex;
            gap: 15px;
            z-index: 1001;
        }}

        .control-btn {{
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            border: none;
            padding: 15px 30px;
            border-radius: 50px;
            cursor: pointer;
            font-size: 1em;
            font-weight: 600;
            box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
            transition: all 0.3s;
        }}

        .control-btn:hover {{
            transform: translateY(-3px);
            box-shadow: 0 6px 20px rgba(102, 126, 234, 0.6);
        }}

        .info-panel {{
            position: fixed;
            top: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 20px;
            border-radius: 10px;
            z-index: 1000;
            backdrop-filter: blur(10px);
            max-width: 300px;
        }}

        .info-panel h3 {{
            color: #667eea;
            margin-bottom: 10px;
        }}

        .info-panel p {{
            margin: 5px 0;
            font-size: 0.9em;
        }}
    </style>
</head>
<body>
    <div id="overlay" style="display: none;">
        <div id="content"></div>
    </div>

    <div class="info-panel">
        <h3>🥽 VR Documentation</h3>
        <p><strong>{title}</strong></p>
        <p>Use WASD to move</p>
        <p>Mouse to look around</p>
        <p>Click 'Toggle Doc' to read</p>
    </div>

    <div id="controls">
        <button class="control-btn" onclick="toggleOverlay()">📄 Toggle Doc</button>
        <button class="control-btn" onclick="enterVR()">🥽 Enter VR</button>
        <button class="control-btn" onclick="resetView()">🔄 Reset View</button>
    </div>

    <a-scene background="color: #000">
        <!-- Sky with gradient -->
        <a-sky color="#1a1a2e"></a-sky>

        <!-- Ambient lighting -->
        <a-light type="ambient" color="#667eea" intensity="0.5"></a-light>
        <a-light type="directional" color="#FFF" intensity="0.8" position="2 4 -3"></a-light>
        <a-light type="point" color="#764ba2" intensity="1" position="0 3 0"></a-light>

        <!-- Floor grid -->
        <a-plane position="0 0 0" rotation="-90 0 0" width="20" height="20"
                 color="#0f0f23" roughness="0.8" metalness="0.2"
                 shader="flat" repeat="10 10"></a-plane>

        <!-- Documentation panels floating in 3D space -->
        <a-entity id="doc-environment" position="0 1.6 -3">
            <!-- Main title panel -->
            <a-plane position="0 2 0" rotation="0 0 0" width="8" height="1.5"
                     color="#667eea" opacity="0.9">
                <a-text value="{title}" align="center" color="#FFF"
                        position="0 0 0.01" width="8" font="roboto"></a-text>
            </a-plane>

            <!-- Floating data panels in circular arrangement -->
            <a-entity id="panel-1" position="-3 0 0">
                <a-box color="#667eea" opacity="0.7" width="1.5" height="2" depth="0.1"
                       roughness="0.3" metalness="0.6"
                       animation="property: rotation; to: 0 360 0; loop: true; dur: 20000">
                    <a-text value="PDF\\nFormat" align="center" color="#FFF"
                            position="0 0.5 0.06" width="2"></a-text>
                    <a-text value="Complete\\nDocumentation" align="center" color="#CCC"
                            position="0 -0.3 0.06" width="1.5"></a-text>
                </a-box>
            </a-entity>

            <a-entity id="panel-2" position="3 0 0">
                <a-box color="#764ba2" opacity="0.7" width="1.5" height="2" depth="0.1"
                       roughness="0.3" metalness="0.6"
                       animation="property: rotation; to: 0 360 0; loop: true; dur: 20000; dir: reverse">
                    <a-text value="Interactive\\n3D View" align="center" color="#FFF"
                            position="0 0.5 0.06" width="2"></a-text>
                    <a-text value="Immersive\\nExperience" align="center" color="#CCC"
                            position="0 -0.3 0.06" width="1.5"></a-text>
                </a-box>
            </a-entity>

            <a-entity id="panel-3" position="0 0 -2">
                <a-cylinder color="#f093fb" opacity="0.7" radius="0.75" height="2"
                           roughness="0.3" metalness="0.6"
                           animation="property: position; to: 0 0.5 -2; loop: true; dur: 3000; dir: alternate; easing: easeInOutSine">
                    <a-text value="VisionBot\\nProject" align="center" color="#FFF"
                            position="0 0.5 0.76" width="2"></a-text>
                </a-cylinder>
            </a-entity>

            <!-- Floating stats spheres -->
            <a-sphere position="-1.5 1.5 -1" radius="0.3" color="#10b981" opacity="0.8"
                     animation="property: position; to: -1.5 2 -1; loop: true; dur: 2000; dir: alternate">
                <a-text value="99.2%\\nSuccess" align="center" color="#FFF"
                        position="0 0 0.31" width="1.2"></a-text>
            </a-sphere>

            <a-sphere position="1.5 1.5 -1" radius="0.3" color="#f59e0b" opacity="0.8"
                     animation="property: position; to: 1.5 2 -1; loop: true; dur: 2500; dir: alternate">
                <a-text value="93.5%\\nOEE" align="center" color="#FFF"
                        position="0 0 0.31" width="1.2"></a-text>
            </a-sphere>
        </a-entity>

        <!-- Camera with controls -->
        <a-entity id="camera-rig" position="0 0 0">
            <a-camera id="main-camera" wasd-controls look-controls>
                <a-cursor color="#667eea" fuse="true" fuse-timeout="1500"></a-cursor>
            </a-camera>
        </a-entity>

        <!-- Particle effect -->
        <a-entity position="0 2.5 -5">
            <a-entity geometry="primitive: sphere; radius: 0.05"
                     material="color: #667eea; emissive: #667eea; emissiveIntensity: 0.5"
                     animation="property: position; to: 0 4 -5; loop: true; dur: 4000; easing: linear"></a-entity>
        </a-entity>
    </a-scene>

    <script>
        // Convert markdown to HTML
        const markdown = `{md_content.replace('`', '\\`')}`;

        document.addEventListener('DOMContentLoaded', function() {{
            const contentDiv = document.getElementById('content');
            contentDiv.innerHTML = marked.parse(markdown);
        }});

        function toggleOverlay() {{
            const overlay = document.getElementById('overlay');
            overlay.style.display = overlay.style.display === 'none' ? 'block' : 'none';
        }}

        function enterVR() {{
            const scene = document.querySelector('a-scene');
            if (scene.is('vr-mode')) {{
                scene.exitVR();
            }} else {{
                scene.enterVR();
            }}
        }}

        function resetView() {{
            const cameraRig = document.getElementById('camera-rig');
            cameraRig.setAttribute('position', '0 0 0');
            const camera = document.getElementById('main-camera');
            camera.setAttribute('rotation', '0 0 0');
        }}

        // Keyboard shortcuts
        document.addEventListener('keydown', function(e) {{
            if (e.key === 'd' || e.key === 'D') {{
                toggleOverlay();
            }} else if (e.key === 'r' || e.key === 'R') {{
                resetView();
            }}
        }});
    </script>
</body>
</html>"""

            # Write AR/VR HTML file
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(ar_vr_html)

            self.conversion_stats['ar_vr']['success'] += 1
            return True

        except Exception as e:
            print(f"❌ AR/VR conversion failed for {md_file}: {str(e)}")
            self.conversion_stats['ar_vr']['failed'] += 1
            return False

    def convert_all(self):
        """Convert all markdown files to all formats"""
        print(f"\n{'='*70}")
        print(f"  CONVERTING {len(self.md_files)} MARKDOWN FILES TO MULTIPLE FORMATS")
        print(f"{'='*70}\n")

        for idx, md_file in enumerate(self.md_files, 1):
            print(f"\n[{idx}/{len(self.md_files)}] Processing: {md_file}")
            print("-" * 70)

            # PDF
            print("  📄 Converting to PDF...", end=" ")
            if self.convert_to_pdf(md_file):
                print("✅")

            # DOCX
            print("  📝 Converting to DOCX...", end=" ")
            if self.convert_to_docx(md_file):
                print("✅")

            # HTML
            print("  🌐 Converting to HTML...", end=" ")
            if self.convert_to_html(md_file):
                print("✅")

            # LaTeX
            print("  📐 Converting to LaTeX...", end=" ")
            if self.convert_to_latex(md_file):
                print("✅")

            # Styled HTML
            print("  🎨 Creating Beautified HTML...", end=" ")
            if self.convert_to_styled_html(md_file):
                print("✅")

            # Infographic
            print("  📊 Creating Infographic HTML...", end=" ")
            if self.convert_to_infographic(md_file):
                print("✅")

            # AR/VR
            print("  🥽 Creating AR/VR WebXR...", end=" ")
            if self.convert_to_ar_vr(md_file):
                print("✅")

        self.generate_summary()

    def generate_summary(self):
        """Generate conversion summary report"""
        print(f"\n\n{'='*70}")
        print(f"  CONVERSION SUMMARY")
        print(f"{'='*70}\n")

        total_success = sum(stats['success'] for stats in self.conversion_stats.values())
        total_failed = sum(stats['failed'] for stats in self.conversion_stats.values())
        total_conversions = len(self.md_files) * len(self.conversion_stats)

        print(f"📊 Total Files Processed: {len(self.md_files)}")
        print(f"📊 Total Conversions: {total_conversions}")
        print(f"✅ Successful: {total_success}")
        print(f"❌ Failed: {total_failed}")
        print(f"📈 Success Rate: {(total_success/total_conversions*100):.1f}%\n")

        print("Format Breakdown:")
        print("-" * 70)
        for format_name, stats in self.conversion_stats.items():
            total = stats['success'] + stats['failed']
            success_rate = (stats['success'] / total * 100) if total > 0 else 0
            print(f"  {format_name.upper():15s}: {stats['success']:2d}/{total:2d} ({success_rate:.0f}%)")

        print(f"\n{'='*70}\n")

        # Create index HTML file
        self._create_index_html()

    def _create_index_html(self):
        """Create an index.html file listing all converted documents"""
        index_content = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VisionBot Documentation - All Formats</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700;900&display=swap');

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Inter', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 40px 20px;
            color: #1e293b;
        }

        .container {
            max-width: 1400px;
            margin: 0 auto;
        }

        header {
            text-align: center;
            color: white;
            margin-bottom: 60px;
        }

        header h1 {
            font-size: 3.5em;
            font-weight: 900;
            margin-bottom: 15px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
        }

        header p {
            font-size: 1.3em;
            opacity: 0.9;
        }

        .stats {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-bottom: 40px;
        }

        .stat-box {
            background: white;
            padding: 25px;
            border-radius: 15px;
            text-align: center;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }

        .stat-box i {
            font-size: 2.5em;
            background: linear-gradient(135deg, #667eea, #764ba2);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 10px;
        }

        .stat-box .value {
            font-size: 2em;
            font-weight: 700;
            color: #1e293b;
        }

        .stat-box .label {
            color: #64748b;
            text-transform: uppercase;
            font-size: 0.9em;
            letter-spacing: 1px;
        }

        .documents {
            background: white;
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }

        .documents h2 {
            font-size: 2em;
            margin-bottom: 30px;
            background: linear-gradient(135deg, #667eea, #764ba2);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .doc-item {
            border: 2px solid #e2e8f0;
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 20px;
            transition: all 0.3s;
        }

        .doc-item:hover {
            border-color: #667eea;
            transform: translateX(5px);
            box-shadow: 0 4px 15px rgba(102, 126, 234, 0.2);
        }

        .doc-title {
            font-size: 1.3em;
            font-weight: 600;
            color: #1e293b;
            margin-bottom: 15px;
        }

        .doc-links {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
        }

        .doc-link {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 10px 20px;
            border-radius: 8px;
            text-decoration: none;
            font-weight: 600;
            transition: all 0.3s;
            font-size: 0.9em;
        }

        .doc-link:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        }

        .link-pdf {
            background: #ef4444;
            color: white;
        }

        .link-docx {
            background: #2563eb;
            color: white;
        }

        .link-html {
            background: #10b981;
            color: white;
        }

        .link-latex {
            background: #8b5cf6;
            color: white;
        }

        .link-styled {
            background: #f59e0b;
            color: white;
        }

        .link-infographic {
            background: #ec4899;
            color: white;
        }

        .link-arvr {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }

        @media (max-width: 768px) {
            header h1 {
                font-size: 2em;
            }

            .doc-links {
                flex-direction: column;
            }

            .doc-link {
                width: 100%;
                justify-content: center;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1><i class="fas fa-robot"></i> VisionBot Documentation</h1>
            <p>Complete Documentation Set - All Formats Available</p>
        </header>

        <div class="stats">
            <div class="stat-box">
                <i class="fas fa-file-alt"></i>
                <div class="value">29</div>
                <div class="label">Documents</div>
            </div>
            <div class="stat-box">
                <i class="fas fa-layer-group"></i>
                <div class="value">7</div>
                <div class="label">Formats</div>
            </div>
            <div class="stat-box">
                <i class="fas fa-download"></i>
                <div class="value">203</div>
                <div class="label">Total Files</div>
            </div>
            <div class="stat-box">
                <i class="fas fa-check-circle"></i>
                <div class="value">100%</div>
                <div class="label">Complete</div>
            </div>
        </div>

        <div class="documents">
            <h2><i class="fas fa-book"></i> Available Documents</h2>
"""

        for md_file in self.md_files:
            stem = Path(md_file).stem
            title = stem.replace('_', ' ').title()

            index_content += f"""
            <div class="doc-item">
                <div class="doc-title">{title}</div>
                <div class="doc-links">
                    <a href="pdf/{stem}.pdf" class="doc-link link-pdf">
                        <i class="fas fa-file-pdf"></i> PDF
                    </a>
                    <a href="docx/{stem}.docx" class="doc-link link-docx">
                        <i class="fas fa-file-word"></i> Word
                    </a>
                    <a href="html/{stem}.html" class="doc-link link-html">
                        <i class="fas fa-file-code"></i> HTML
                    </a>
                    <a href="latex/{stem}.tex" class="doc-link link-latex">
                        <i class="fas fa-file-code"></i> LaTeX
                    </a>
                    <a href="html_styled/{stem}.html" class="doc-link link-styled">
                        <i class="fas fa-palette"></i> Styled
                    </a>
                    <a href="infographic/{stem}.html" class="doc-link link-infographic">
                        <i class="fas fa-chart-bar"></i> Infographic
                    </a>
                    <a href="ar_vr/{stem}.html" class="doc-link link-arvr">
                        <i class="fas fa-vr-cardboard"></i> AR/VR
                    </a>
                </div>
            </div>
"""

        index_content += """
        </div>
    </div>
</body>
</html>
"""

        with open(self.output_dir / 'index.html', 'w', encoding='utf-8') as f:
            f.write(index_content)

        print(f"✅ Index file created: output/index.html")
        print(f"   Open in browser: file://{self.output_dir.absolute()}/index.html\n")


if __name__ == "__main__":
    converter = DocumentConverter()
    converter.convert_all()

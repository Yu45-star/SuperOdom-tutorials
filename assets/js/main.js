// SuperOdom Tutorial Website - Main JavaScript File

(function () {
    'use strict';

    // 🚀 Execute after page load
    document.addEventListener('DOMContentLoaded', function () {

        // Initialize all features
        initSmoothScrolling();
        initSearchFunctionality();
        initThemeToggle();
        initProgressBar();
        initLazyLoading();
        initTooltips();

        console.log('🎉 SuperOdom Tutorial website loaded successfully!');
    });

    // 📜 Smooth Scrolling Feature
    function initSmoothScrolling() {
        const links = document.querySelectorAll('a[href^="#"]');

        links.forEach(link => {
            link.addEventListener('click', function (e) {
                const href = this.getAttribute('href');

                // Skip empty links
                if (href === '#') return;

                const target = document.querySelector(href);
                if (target) {
                    e.preventDefault();

                    target.scrollIntoView({
                        behavior: 'smooth',
                        block: 'start'
                    });

                    // Update URL without triggering page jump
                    history.pushState(null, null, href);
                }
            });
        });
    }

    // 🔍 Search Functionality (Enhanced)
    function initSearchFunctionality() {
        const searchInput = document.getElementById('search-input');
        const searchResults = document.getElementById('search-results');

        if (!searchInput) return;

        // Debounce function
        function debounce(func, wait) {
            let timeout;
            return function executedFunction(...args) {
                const later = () => {
                    clearTimeout(timeout);
                    func(...args);
                };
                clearTimeout(timeout);
                timeout = setTimeout(later, wait);
            };
        }

        // Search function
        const performSearch = debounce(function (query) {
            if (query.length < 2) {
                searchResults.innerHTML = '';
                searchResults.style.display = 'none';
                return;
            }

            // Simple client-side search
            const searchableElements = document.querySelectorAll('h1, h2, h3, p, .tutorial-card');
            const results = [];

            searchableElements.forEach(element => {
                const text = element.textContent.toLowerCase();
                if (text.includes(query.toLowerCase())) {
                    results.push({
                        element: element,
                        text: element.textContent.trim(),
                        type: element.tagName.toLowerCase()
                    });
                }
            });

            displaySearchResults(results, query);
        }, 300);

        // Display search results
        function displaySearchResults(results, query) {
            if (results.length === 0) {
                searchResults.innerHTML = `
                    <div class="search-result-item">
                        <small class="text-muted">No results found for "${query}"</small>
                    </div>
                `;
            } else {
                const resultsHTML = results.slice(0, 5).map(result => `
                    <div class="search-result-item" onclick="scrollToElement('${result.element.id || 'top'}')">
                        <strong>${highlightQuery(result.text.substring(0, 100), query)}...</strong>
                        <br><small class="text-muted">${result.type.toUpperCase()}</small>
                    </div>
                `).join('');

                searchResults.innerHTML = resultsHTML;
            }

            searchResults.style.display = 'block';
        }

        // Highlight search query
        function highlightQuery(text, query) {
            const regex = new RegExp(`(${query})`, 'gi');
            return text.replace(regex, '<mark>$1</mark>');
        }

        // Scroll to element
        window.scrollToElement = function (elementId) {
            const element = document.getElementById(elementId);
            if (element) {
                element.scrollIntoView({ behavior: 'smooth' });
            }
            searchResults.style.display = 'none';
            searchInput.value = '';
        };

        // Bind search event
        searchInput.addEventListener('input', function () {
            performSearch(this.value);
        });

        // Click outside to close search results
        document.addEventListener('click', function (e) {
            if (!searchInput.contains(e.target) && !searchResults.contains(e.target)) {
                searchResults.style.display = 'none';
            }
        });
    }

    // 🌙 Theme Toggle Functionality
    function initThemeToggle() {
        // Check local storage for theme preference
        const savedTheme = localStorage.getItem('theme');
        const systemTheme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
        const currentTheme = savedTheme || systemTheme;

        // Apply theme
        document.documentElement.setAttribute('data-theme', currentTheme);

        // Create theme toggle button if it doesn't exist
        let themeToggle = document.getElementById('theme-toggle');
        if (!themeToggle) {
            themeToggle = document.createElement('button');
            themeToggle.id = 'theme-toggle';
            themeToggle.className = 'btn btn-outline-light btn-sm';
            themeToggle.innerHTML = currentTheme === 'dark' ? '☀️' : '🌙';
            themeToggle.title = 'Toggle theme';

            // Add to navbar
            const navbar = document.querySelector('.navbar-nav');
            if (navbar) {
                const li = document.createElement('li');
                li.className = 'nav-item';
                li.appendChild(themeToggle);
                navbar.appendChild(li);
            }
        }

        // Theme toggle event
        themeToggle.addEventListener('click', function () {
            const currentTheme = document.documentElement.getAttribute('data-theme');
            const newTheme = currentTheme === 'dark' ? 'light' : 'dark';

            document.documentElement.setAttribute('data-theme', newTheme);
            localStorage.setItem('theme', newTheme);

            this.innerHTML = newTheme === 'dark' ? '☀️' : '🌙';
        });
    }

    // 📊 Reading Progress Bar
    function initProgressBar() {
        // Create progress bar
        const progressBar = document.createElement('div');
        progressBar.id = 'reading-progress';
        progressBar.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 0%;
            height: 3px;
            background: linear-gradient(90deg, var(--primary-color), var(--accent-color));
            z-index: 9999;
            transition: width 0.3s ease;
        `;
        document.body.appendChild(progressBar);

        // Update progress
        function updateProgress() {
            const scrollTop = window.pageYOffset;
            const docHeight = document.body.scrollHeight - window.innerHeight;
            const scrollPercent = (scrollTop / docHeight) * 100;

            progressBar.style.width = Math.min(scrollPercent, 100) + '%';
        }

        // Bind scroll event
        window.addEventListener('scroll', updateProgress);

        // Only show on tutorial pages
        if (!document.querySelector('.tutorial')) {
            progressBar.style.display = 'none';
        }
    }

    // 🖼️ Image Lazy Loading
    function initLazyLoading() {
        const images = document.querySelectorAll('img[data-src]');

        const imageObserver = new IntersectionObserver((entries, observer) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    const img = entry.target;
                    img.src = img.dataset.src;
                    img.classList.remove('lazy');
                    observer.unobserve(img);
                }
            });
        });

        images.forEach(img => {
            imageObserver.observe(img);
        });
    }

    // 💡 Tooltip Functionality
    function initTooltips() {
        // Add tooltips to all elements with title attribute
        const tooltipElements = document.querySelectorAll('[title]');

        tooltipElements.forEach(element => {
            element.addEventListener('mouseenter', function (e) {
                const title = this.getAttribute('title');
                if (!title) return;

                // Create tooltip
                const tooltip = document.createElement('div');
                tooltip.className = 'custom-tooltip';
                tooltip.textContent = title;
                tooltip.style.cssText = `
                    position: absolute;
                    background: rgba(0,0,0,0.8);
                    color: white;
                    padding: 8px 12px;
                    border-radius: 6px;
                    font-size: 12px;
                    z-index: 10000;
                    pointer-events: none;
                    white-space: nowrap;
                `;

                document.body.appendChild(tooltip);

                // Remove original title to avoid duplicate display
                this.setAttribute('data-original-title', title);
                this.removeAttribute('title');

                // Position tooltip
                const rect = this.getBoundingClientRect();
                tooltip.style.left = rect.left + (rect.width / 2) - (tooltip.offsetWidth / 2) + 'px';
                tooltip.style.top = rect.top - tooltip.offsetHeight - 8 + 'px';
            });

            element.addEventListener('mouseleave', function () {
                const tooltip = document.querySelector('.custom-tooltip');
                if (tooltip) {
                    tooltip.remove();
                }

                // Restore title attribute
                const originalTitle = this.getAttribute('data-original-title');
                if (originalTitle) {
                    this.setAttribute('title', originalTitle);
                    this.removeAttribute('data-original-title');
                }
            });
        });
    }

    // 🔄 Back to Top Button
    function initBackToTop() {
        const backToTopBtn = document.createElement('button');
        backToTopBtn.innerHTML = '<i class="fas fa-chevron-up"></i>';
        backToTopBtn.className = 'btn btn-primary back-to-top';
        backToTopBtn.style.cssText = `
            position: fixed;
            bottom: 20px;
            right: 20px;
            width: 50px;
            height: 50px;
            border-radius: 50%;
            display: none;
            z-index: 1000;
            box-shadow: 0 4px 15px rgba(0,0,0,0.2);
        `;

        document.body.appendChild(backToTopBtn);

        // Show/Hide button
        window.addEventListener('scroll', function () {
            if (window.pageYOffset > 300) {
                backToTopBtn.style.display = 'block';
            } else {
                backToTopBtn.style.display = 'none';
            }
        });

        // Click to scroll to top
        backToTopBtn.addEventListener('click', function () {
            window.scrollTo({
                top: 0,
                behavior: 'smooth'
            });
        });
    }

    // Initialize back to top button
    initBackToTop();

    // 🎯 Code Block Enhancement
    function enhanceCodeBlocks() {
        const codeBlocks = document.querySelectorAll('pre code');

        codeBlocks.forEach(codeBlock => {
            const pre = codeBlock.parentNode;

            // Add language label
            const language = codeBlock.className.match(/language-(\w+)/);
            if (language) {
                const langLabel = document.createElement('span');
                langLabel.textContent = language[1].toUpperCase();
                langLabel.className = 'code-lang-label';
                langLabel.style.cssText = `
                    position: absolute;
                    top: 10px;
                    left: 15px;
                    background: rgba(255,255,255,0.1);
                    color: rgba(255,255,255,0.7);
                    padding: 2px 8px;
                    border-radius: 4px;
                    font-size: 11px;
                    font-weight: bold;
                `;
                pre.appendChild(langLabel);
            }

            // Add line numbers (optional)
            if (codeBlock.textContent.split('\n').length > 3) {
                const lines = codeBlock.textContent.split('\n');
                const lineNumbers = document.createElement('div');
                lineNumbers.className = 'line-numbers';
                lineNumbers.style.cssText = `
                    position: absolute;
                    left: 0;
                    top: 0;
                    bottom: 0;
                    width: 40px;
                    background: rgba(0,0,0,0.1);
                    border-right: 1px solid rgba(255,255,255,0.1);
                    padding: 1.5rem 0;
                    text-align: center;
                    font-size: 12px;
                    color: rgba(255,255,255,0.5);
                `;

                const lineNumbersHTML = lines.map((_, index) =>
                    `<div style="line-height: 1.4;">${index + 1}</div>`
                ).join('');
                lineNumbers.innerHTML = lineNumbersHTML;

                pre.style.paddingLeft = '50px';
                pre.appendChild(lineNumbers);
            }
        });
    }

    // Initialize code block enhancement
    enhanceCodeBlocks();

})();

// 🌐 Global Utility Functions
window.SuperOdomUtils = {
    // Smooth scroll to element
    scrollToElement: function (selector) {
        const element = document.querySelector(selector);
        if (element) {
            element.scrollIntoView({ behavior: 'smooth' });
        }
    },

    // Copy text to clipboard
    copyToClipboard: function (text) {
        if (navigator.clipboard) {
            return navigator.clipboard.writeText(text);
        } else {
            // Fallback method
            const textArea = document.createElement('textarea');
            textArea.value = text;
            document.body.appendChild(textArea);
            textArea.select();
            document.execCommand('copy');
            document.body.removeChild(textArea);
            return Promise.resolve();
        }
    },

    // Format date
    formatDate: function (date) {
        return new Intl.DateTimeFormat('en-US', {
            year: 'numeric',
            month: 'long',
            day: 'numeric'
        }).format(new Date(date));
    }
};
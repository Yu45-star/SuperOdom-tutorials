// 🧭 Navigation bar scrolling effect
function initHeaderScrollEffect() {
    const header = document.querySelector('.site-header');

    if (!header) return;

    window.addEventListener('scroll', function () {
        if (window.scrollY > 50) {
            header.classList.add('scrolled');
        } else {
            header.classList.remove('scrolled');
        }
    });
}

// 🎯 Active navigation link highlighting
function initActiveNavigation() {
    const navLinks = document.querySelectorAll('.nav-link');
    const currentPath = window.location.pathname;

    navLinks.forEach(link => {
        const href = link.getAttribute('href');
        if (href && (currentPath === href || currentPath.startsWith(href + '/'))) {
            link.classList.add('active');
        }
    });
}

// 🎨 Theme toggle enhancement
function enhanceThemeToggle() {
    const themeToggle = document.getElementById('theme-toggle');
    if (!themeToggle) return;

    // Update icon
    function updateThemeIcon(theme) {
        const icon = themeToggle.querySelector('i');
        if (icon) {
            icon.className = theme === 'dark' ? 'fas fa-sun' : 'fas fa-moon';
        }
    }

    // Initialize icon
    const currentTheme = document.documentElement.getAttribute('data-theme') || 'light';
    updateThemeIcon(currentTheme);

    // Listen for theme changes
    const observer = new MutationObserver(function (mutations) {
        mutations.forEach(function (mutation) {
            if (mutation.type === 'attributes' && mutation.attributeName === 'data-theme') {
                const newTheme = document.documentElement.getAttribute('data-theme');
                updateThemeIcon(newTheme);
            }
        });
    });

    observer.observe(document.documentElement, {
        attributes: true,
        attributeFilter: ['data-theme']
    });
}

// 🌙 Theme Toggle Functionality
function initThemeToggle() {
    console.log('🔧 Initializing theme toggle...');

    // Get theme toggle button
    const themeToggle = document.getElementById('theme-toggle');

    if (!themeToggle) {
        console.error('❌ Theme toggle button not found!');
        return;
    }

    console.log('✅ Theme toggle button found');

    // Check for saved theme preference in localStorage
    const savedTheme = localStorage.getItem('superodom-theme');
    const systemTheme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    const currentTheme = savedTheme || systemTheme;

    console.log('💾 Saved theme:', savedTheme);
    console.log('🖥️ System theme:', systemTheme);
    console.log('🎨 Current theme:', currentTheme);

    // Apply the theme
    function applyTheme(theme) {
        console.log('🎨 Applying theme:', theme);

        // Set HTML attributes
        document.documentElement.setAttribute('data-theme', theme);
        document.body.setAttribute('data-theme', theme);

        // Update button icon
        const icon = themeToggle.querySelector('i');
        if (icon) {
            icon.className = theme === 'dark' ? 'fas fa-sun' : 'fas fa-moon';
        }

        // Save to localStorage
        localStorage.setItem('superodom-theme', theme);

        console.log('✅ Theme applied successfully');
    }

    // Initialize theme
    applyTheme(currentTheme);

    // Theme toggle event listener
    themeToggle.addEventListener('click', function (e) {
        e.preventDefault();
        console.log('🖱️ Theme toggle clicked');

        const currentTheme = document.documentElement.getAttribute('data-theme') || 'light';
        const newTheme = currentTheme === 'dark' ? 'light' : 'dark';

        console.log('🔄 Switching from', currentTheme, 'to', newTheme);

        applyTheme(newTheme);

        // Add click feedback
        this.style.transform = 'scale(0.95)';
        setTimeout(() => {
            this.style.transform = 'scale(1)';
        }, 150);
    });

    console.log('🎉 Theme toggle initialized successfully');
}

// Add the DOMContentLoaded event listener to initialize functions
document.addEventListener('DOMContentLoaded', function () {
    console.log('📄 DOM Content Loaded');

    // Delay to ensure all elements are fully loaded
    setTimeout(() => {
        initThemeToggle();
    }, 100);
    
    // Existing initialization functions...
    initSmoothScrolling();
    initSearchFunctionality();
    initThemeToggle();
    initProgressBar();
    initLazyLoading();
    initTooltips();

    // New functions
    initHeaderScrollEffect();
    initActiveNavigation();
    enhanceThemeToggle();

    console.log('🎉 SuperOdom Tutorial website loaded successfully!');
});
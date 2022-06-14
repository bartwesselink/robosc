const vscode = acquireVsCodeApi();

const updateActiveValue = (element, value) => {
    const items = element.querySelectorAll('.middleware-option');

    items.forEach(item => {
        item.classList.remove('active');

        if (item.innerText.trim() === value.trim()) {
            item.classList.add('active');
        }
    });

    vscode.postMessage({
        command: 'middleware',
        data: value,
    });
}

export const initializeMiddlewareToggle = (settings, reset) => {
    const element = document.querySelector('.middleware-switcher');
    const items = element.querySelectorAll('.middleware-option');

    const currentValue = settings.default;
    updateActiveValue(element, currentValue);

    items.forEach(it => {
        it.addEventListener('click', () => {
            reset();
            updateActiveValue(element, it.innerText);
        });
    });
}

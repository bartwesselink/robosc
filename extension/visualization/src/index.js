import './assets/main.scss'
import { updateState, visualizeComponent } from './component';
import { initializeMiddlewareToggle } from './middleware-toggle';

let lastDefinition = null;
let lastSerialized = null;

const root = document.getElementById('root');

const reset = () => {
    lastDefinition = null;
    lastSerialized = null;
    root.innerText = 'Connecting to controller...';
};

reset();

const handleData = (data) => {
    const current = JSON.stringify(data.definition);

    if (lastSerialized !== current) {
        root.innerHTML = '';

        const definition = data.definition;

        const title = document.createElement('h1');
        title.innerHTML = definition.name;

        const grid = document.createElement('div');
        grid.classList.add('grid');

        root.appendChild(title);
        root.appendChild(grid);

        for (const component of definition.components) {
            grid.appendChild(visualizeComponent(component, data.current[component.name]));
        }

        lastSerialized = current;
        lastDefinition = definition;
    }

    setTimeout(() => updateState(lastDefinition, data.current, data.transitions));
};

window.addEventListener('message', event => {
    const wrapper = event.data;

    switch (wrapper.command) {
        case 'visualize':
            return handleData(wrapper.data);
        case 'settings':
            return initializeMiddlewareToggle(wrapper.data, () => reset());
    }
});
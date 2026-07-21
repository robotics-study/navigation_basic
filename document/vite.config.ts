import {defineConfig} from 'vite';


export default defineConfig(({mode}) => {
    return {
        base: mode == 'production' ? '/navigation' : '/',
        server: {
            host: true,
            port: 3000
        }
    }
});

// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import starlightLinksValidator from 'starlight-links-validator';

// Served by GitHub Pages as a project site: https://matthewyjiang.github.io/lantern/.
// Links in Markdown don't get `base` added for you, so write them as /lantern/... .
export default defineConfig({
	site: 'https://matthewyjiang.github.io',
	base: '/lantern',
	integrations: [
		starlight({
			title: 'Lantern',
			description:
				'A small RViz stand-in in Rust. Draws point clouds, TF, poses and a robot model from any Foxglove WebSocket server.',
			logo: { src: './src/assets/lantern-mark.svg' },
			favicon: '/favicon.svg',
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/matthewyjiang/lantern' }],
			customCss: ['./src/styles/theme.css'],
			// Fails the build on a broken internal link or #anchor.
			plugins: [starlightLinksValidator()],
			sidebar: [
				{
					label: 'Start here',
					items: ['getting-started', 'connect'],
				},
				{
					label: 'Guides',
					items: ['guides/settings', 'guides/robot-model', 'guides/kpd'],
				},
				{
					label: 'Reference',
					items: ['reference/topics', 'reference/architecture', 'reference/performance'],
				},
			],
		}),
	],
});

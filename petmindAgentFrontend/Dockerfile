FROM node:22-alpine AS build
WORKDIR /app
ENV PNPM_HOME=/pnpm
ENV PATH=$PNPM_HOME:$PATH
COPY package.json pnpm-lock.yaml ./
# Pin pnpm to the version declared in package.json#packageManager so Docker and
# local installs resolve the lockfile identically.
RUN corepack enable && corepack install
RUN pnpm install --frozen-lockfile
COPY index.html vite.config.ts tsconfig.json tsconfig.app.json tsconfig.node.json ./
COPY public ./public
COPY src ./src
ARG VITE_API_ROOT=
ENV VITE_API_ROOT=$VITE_API_ROOT
RUN pnpm build

FROM nginx:1.30.4-alpine
ENV AGENT_UPSTREAM=http://host.docker.internal:8002
ENV NGINX_ENVSUBST_FILTER=AGENT_UPSTREAM
COPY nginx.conf /etc/nginx/templates/default.conf.template
COPY security-headers.conf /etc/nginx/snippets/petmind-security-headers.conf
COPY --from=build /app/dist /usr/share/nginx/html
EXPOSE 80
HEALTHCHECK --interval=15s --timeout=3s --start-period=5s --retries=3 \
    CMD wget -qO- http://127.0.0.1/healthz >/dev/null || exit 1
